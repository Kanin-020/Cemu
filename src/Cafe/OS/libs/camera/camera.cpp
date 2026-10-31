#include "Common/precompiled.h"
#include "Cafe/OS/common/OSCommon.h"
#include "camera.h"
#include "Cafe/OS/RPL/rpl.h"
#include "Cafe/OS/libs/coreinit/coreinit_Alarm.h"
#include "Cafe/OS/libs/coreinit/coreinit_Time.h"
#include "Cafe/HW/Espresso/PPCCallback.h"
#include "Cafe/Twinpad/TwinpadCameraDecoder.h"
#include "Cafe/Twinpad/TwinpadClient.h"

namespace camera
{

	struct CAMInitInfo_t
	{
		/* +0x00 */ uint32be ukn00;
		/* +0x04 */ uint32be width;
		/* +0x08 */ uint32be height;
		
		/* +0x0C */ uint32be workMemorySize;
		/* +0x10 */ MEMPTR<void> workMemory;

		/* +0x14 */ uint32be handlerFuncPtr;

		/* +0x18 */ uint32be ukn18;
		/* +0x1C */ uint32be fps;

		/* +0x20 */ uint32be ukn20;
	};

	struct CAMTargetSurface 
	{
		// Layout real del guest según WiiUBrew (Camera.rpl, struct CAMSurface):
		/* +0x00 */ uint32be surfaceSize;
		/* +0x04 */ MEMPTR<void> surfacePtr;
		/* +0x08 */ uint32be surfaceHeight;   // altura del surface (480 en Nintendo Land)
		/* +0x0C */ uint32be surfaceWidth;    // ancho visible (640)
		/* +0x10 */ uint32be surfacePitch;    // stride de fila (768 = 640 + padding)
		/* +0x14 */ uint32be surfaceAlignment;
		/* +0x18 */ uint32be surfaceTileMode;
		/* +0x1C */ uint32be ukn1C;
	};

	struct CAMCallbackParam 
	{
		// type 0 - frame decoded | field1 - imagePtr, field2 - imageSize, field3 - ukn (0)
		// type 1 - ???


		/* +0x0 */ uint32be type; // 0 -> Frame decoded
		/* +0x4 */ uint32be field1;
		/* +0x8 */ uint32be field2;
		/* +0xC */ uint32be field3;
	};


	#define CAM_ERROR_SUCCESS			0
	#define CAM_ERROR_INVALID_HANDLE		-8

	std::vector<struct CameraInstance*> g_table_cameraHandles;
	std::vector<struct CameraInstance*> g_activeCameraInstances;
	std::recursive_mutex g_mutex_camera;
	std::atomic_int g_cameraCounter{ 0 };
	SysAllocator<coreinit::OSAlarm_t, 1> g_alarm_camera;
	SysAllocator<CAMCallbackParam, 1> g_cameraHandlerParam;

	CameraInstance* GetCameraInstanceByHandle(sint32 camHandle)
	{
		std::unique_lock<std::recursive_mutex> _lock(g_mutex_camera);
		// CR-046: el juego opera 0-based para el sometimiento de superficies
		// ("cámara 0" = primera) mientras Init devuelve 1-based. Resolve:
		// handle 0 → tabla[0]; handle N → tabla[N-1].
		if (camHandle < 0)
			return nullptr;
		const size_t tableIndex = camHandle == 0 ? 0 : static_cast<size_t>(camHandle - 1);
		if (tableIndex >= g_table_cameraHandles.size())
			return nullptr;
		return g_table_cameraHandles[tableIndex];
	}

	// CR-038: traduce la cantidad de instancias de cámara abiertas a la
	// demanda vigente del video del teléfono. Solo habla en transiciones.
	static std::atomic_bool s_cameraDemandSent{ false };

	// Diagnóstico CR-038: entrada común de registro del HLE. Throttle 1 Hz
	// por “línea” para no inundar el log con polls del juego (CAMClose a 30 Hz).
	static std::atomic<uint64_t> s_camLogLastMs{ 0 };
	static void LogCamThrottled(std::string_view line)
	{
		const auto nowMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count());
		uint64_t previous = s_camLogLastMs.load();
		if (nowMs - previous < 1000)
			return;
		if (!s_camLogLastMs.compare_exchange_strong(previous, nowMs))
			return;
		cemuLog_log(LogType::Force, "{}", line);
	}

	void NotifyCameraDemand(bool on)
	{
		if (s_cameraDemandSent.exchange(on) == on)
			return;
		TwinpadClient::Instance().PublishCameraRequest(on);
		cemuLog_log(LogType::Force, "[TwinpadCamera] camera_request on={}", on);
	}

	struct CameraInstance 
	{
		CameraInstance(uint32 frameWidth, uint32 frameHeight, MPTR handlerFunc) : width(frameWidth), height(frameHeight), handlerFunc(handlerFunc) { AcquireHandle(); };
		~CameraInstance() { if (isOpen) { CloseCam(); } ReleaseHandle(); };

		sint32 handle{ 0 };
		uint32 width;
		uint32 height;
		bool isOpen{false};
		std::queue<CAMTargetSurface> queue_targetSurfaces;
		MPTR handlerFunc;

		bool OpenCam()
		{
			if (isOpen)
				return false;
			isOpen = true;
			bool notify = false;
			{
				std::unique_lock<std::recursive_mutex> _lock(g_mutex_camera);
				// CR-046: la instancia ya consta como activa desde Init;
				// OpenCam solo la añade si no estaba (sin duplicados).
				bool alreadyActive = false;
				for (auto* inst : g_activeCameraInstances)
				{
					if (inst == this)
					{
						alreadyActive = true;
						break;
					}
				}
				if (!alreadyActive)
					g_activeCameraInstances.push_back(this);
				notify = g_activeCameraInstances.size() == 1; // 0 → 1 instancias
			}
			if (notify)
				NotifyCameraDemand(true); // CR-038: pedir la cámara del teléfono
			return true;
		}

		bool CloseCam()
		{
			if (!isOpen)
				return false;
			isOpen = false;
			bool notify = false;
			{
				std::unique_lock<std::recursive_mutex> _lock(g_mutex_camera);
				vectorRemoveByValue(g_activeCameraInstances, this);
				notify = g_activeCameraInstances.empty(); // de vuelta a 0
			}
			if (notify)
				NotifyCameraDemand(false); // CR-038: dejar de enviar la cámara
			return true;
		}

		void QueueTargetSurface(CAMTargetSurface* targetSurface)
		{
			std::unique_lock<std::recursive_mutex> _lock(g_mutex_camera);
			cemu_assert_debug(queue_targetSurfaces.size() < 100); // check for sane queue length
			queue_targetSurfaces.push(*targetSurface);
		}

	private:
		void AcquireHandle()
		{
			std::unique_lock<std::recursive_mutex> _lock(g_mutex_camera);
			for (uint32 i = 0; i < g_table_cameraHandles.size(); i++)
			{
				if (g_table_cameraHandles[i] == nullptr)
				{
					g_table_cameraHandles[i] = this;
					this->handle = i + 1;
					return;
				}
			}
			this->handle = (sint32)(g_table_cameraHandles.size() + 1);
			g_table_cameraHandles.push_back(this);
		}

		void ReleaseHandle()
		{
			for (uint32 i = 0; i < g_table_cameraHandles.size(); i++)
			{
				if (g_table_cameraHandles[i] == this)
				{
					g_table_cameraHandles[i] = nullptr;
					return;
				}
			}
			cemu_assert_debug(false);
		}
	};

	sint32 CAMGetMemReq(void* ukn)
	{
		LogCamThrottled("[TwinpadCamera] CAMGetMemReq");
		return 1 * 1024; // always return 1KB
	}

	sint32 CAMCheckMemSegmentation(void* base, uint32 size)
	{
		LogCamThrottled("[TwinpadCamera] CAMCheckMemSegmentation");
		return CAM_ERROR_SUCCESS; // always return success
	}

	void ppcCAMUpdate60(PPCInterpreter_t* hCPU)
	{
		// update all open camera instances
		size_t numCamInstances = g_activeCameraInstances.size();
		//for (auto& itr : g_activeCameraInstances)
		for(size_t i=0; i<numCamInstances; i++)
		{
			std::unique_lock<std::recursive_mutex> _lock(g_mutex_camera);
			if (i >= g_activeCameraInstances.size())
				break;
			CameraInstance* camInstance = g_activeCameraInstances[i];
			// todo - handle 30 / 60 FPS
			if (camInstance->queue_targetSurfaces.empty())
				continue;
			auto& targetSurface = camInstance->queue_targetSurfaces.front();
			// CR-034: rellena la superficie con el último frame NV12 de la cámara del teléfono.
			uint8_t* surface = static_cast<uint8_t*>(targetSurface.surfacePtr.GetPtr());
			size_t copied = 0;
			if (surface)
			{
				// CR-046: diagnóstico del formato real del surface. En el primer
				// llenado de cada buffer del juego volcamos bytes clave ANTES de
				// escribir: un pre-relleno con 0x80 (croma neutro) delata el layout.
				static std::vector<uint32> s_dumpedSurfaces;
				const uint32 surfaceAddr = targetSurface.surfacePtr.GetMPTR();
				if (std::find(s_dumpedSurfaces.begin(), s_dumpedSurfaces.end(), surfaceAddr)
					== s_dumpedSurfaces.end())
				{
					s_dumpedSurfaces.push_back(surfaceAddr);
					auto dump = [&surface](const char* tag, size_t offset)
					{
						uint8 bytes[16];
						std::memcpy(bytes, surface + offset, sizeof(bytes));
						cemuLog_log(LogType::Force, "[TwinpadCamera] prefill {} @{}: {}",
							tag, offset, fmt::format("{:02x}", fmt::join(bytes, " ")));
					};
					dump("head", 0);
					dump("y-480x640", 307200);
					dump("uv-480x640", 460800);
					dump("y-576x640", 368640);
				}

				// CR-046 + WiiUBrew: la API entrega el surface en NV12, y el
				// propio juego rellena height/width/pitch en la CAMSurface
				// (Nintendo Land: 480×640 visible, pitch 768, buffer 552960).
				// El llenado usa esos campos; si no son válidos, cae a la
				// geometría pedida en CAMInit con pitch = width.
				const uint32 height = targetSurface.surfaceHeight;
				const uint32 width = targetSurface.surfaceWidth;
				const uint32 pitch = targetSurface.surfacePitch;
				uint32 fillHeight = camInstance->height;
				uint32 fillWidth = camInstance->width;
				uint32 fillPitch = fillWidth;
				if (height != 0 && width != 0 && pitch >= width
					&& pitch % 2 == 0 && (size_t)pitch * height * 3 / 2 <= (size_t)targetSurface.surfaceSize)
				{
					fillHeight = height;
					fillWidth = width;
					fillPitch = pitch;
				}
				cemuLog_log(LogType::Force, "[TwinpadCamera] fill geometry surface={:04d}x{:04d} pitch={:04d} (guest h={} w={} p={})",
					fillWidth, fillHeight, fillPitch, height, width, pitch);
				if ((size_t)fillPitch * fillHeight * 3 / 2 <= (size_t)targetSurface.surfaceSize)
					copied = TwinpadCameraDecoder::Instance().ScaleLatestNv12(
						surface, targetSurface.surfaceSize, fillPitch, fillHeight, fillWidth);
				if (copied == 0)
					copied = TwinpadCameraDecoder::Instance().CopyLatestNv12(
						surface, targetSurface.surfaceSize);
			}
			// Diagnóstico CR-046: si copied=0, la pantalla del juego nunca recibe el frame.
			LogCamThrottled(fmt::format("[TwinpadCamera] fill surface ptr={:08X} size={} frame={} copied={}",
				(uint32)targetSurface.surfacePtr.GetMPTR(), (uint32)targetSurface.surfaceSize,
				TwinpadCameraDecoder::Instance().HasFrame() ? 1 : 0, copied));
			g_cameraHandlerParam->type = 0;
			g_cameraHandlerParam->field1 = targetSurface.surfacePtr.GetMPTR();
			g_cameraHandlerParam->field2 = targetSurface.surfaceSize;
			g_cameraHandlerParam->field3 = 0;
			cemu_assert_debug(camInstance->handlerFunc != MPTR_NULL);
			camInstance->queue_targetSurfaces.pop();
			_lock.unlock();
			PPCCoreCallback(camInstance->handlerFunc, g_cameraHandlerParam.GetPtr());
		}
		osLib_returnFromFunction(hCPU, 0);
	}


	sint32 CAMInit(uint32 cameraId, CAMInitInfo_t* camInitInfo, uint32be* error)
	{
		// Diagnóstico CR-038: evidencia de que el juego toca el HLE de cámara.
		// Si no aparece este log, Nintendo Land nunca llega a usar la cámara.
		cemuLog_log(LogType::Force, "[TwinpadCamera] CAMInit id={} {}x{}",
		cameraId, (uint32)camInitInfo->width, (uint32)camInitInfo->height);
		CameraInstance* camInstance = new CameraInstance(camInitInfo->width, camInitInfo->height, camInitInfo->handlerFuncPtr);
		*error = 0; // Hunter's Trophy 2 will fail to boot if we don't set this
		std::unique_lock<std::recursive_mutex> _lock(g_mutex_camera);
		if (g_cameraCounter == 0)
		{
			coreinit::OSCreateAlarm(g_alarm_camera.GetPtr());
			coreinit::OSSetPeriodicAlarm(g_alarm_camera.GetPtr(), coreinit::OSGetTime(), (uint64)ESPRESSO_TIMER_CLOCK / 60ull, RPLLoader_MakePPCCallable(ppcCAMUpdate60));
		}
		// CR-046: la instancia entra activa al crear (el juego somete
		// superficies directamente tras Init; nunca llama CAMOpen).
		g_activeCameraInstances.push_back(camInstance);
		g_cameraCounter++;
		if (g_cameraCounter == 1)
			NotifyCameraDemand(true); // CR-046: demanda en la creación, no en Open

		return camInstance->handle;
	}

	sint32 CAMExit(sint32 camHandle)
	{
		CameraInstance* camInstance = GetCameraInstanceByHandle(camHandle);
		if (!camInstance)
			return CAM_ERROR_INVALID_HANDLE;
		cemuLog_log(LogType::Force, "[TwinpadCamera] CAMExit handle={}", camHandle);
		CAMClose(camHandle);
		{
			std::unique_lock<std::recursive_mutex> _lock(g_mutex_camera);
			// CR-046: la instancia sale de la lista activa en Exit (antes
			// quedaba un puntero colgante en el estado global).
			vectorRemoveByValue(g_activeCameraInstances, camInstance);
			g_cameraCounter--;
			if (g_cameraCounter == 0)
				coreinit::OSCancelAlarm(g_alarm_camera.GetPtr());
		}
		delete camInstance;
		return CAM_ERROR_SUCCESS;
	}

	sint32 CAMOpen(sint32 camHandle)
	{
		CameraInstance* camInstance = GetCameraInstanceByHandle(camHandle);
		if (!camInstance)
		{
			cemuLog_log(LogType::Force, "[TwinpadCamera] CAMOpen handle={} INVALIDO", camHandle);
			return CAM_ERROR_INVALID_HANDLE;
		}
		cemuLog_log(LogType::Force, "[TwinpadCamera] CAMOpen handle={}", camHandle);
		camInstance->OpenCam();
		return CAM_ERROR_SUCCESS;
	}

	sint32 CAMClose(sint32 camHandle)
	{
		CameraInstance* camInstance = GetCameraInstanceByHandle(camHandle);
		if (!camInstance)
		{
			cemuLog_log(LogType::Force, "[TwinpadCamera] CAMClose handle={} INVALIDO", camHandle);
			return CAM_ERROR_INVALID_HANDLE;
		}
		// Diagnóstico: Nintendo Land entra CAMInit y luego CAMClose a ~30 Hz
		// sin CAMOpen. Agregamos el estado y agregamos el conteo por segundo
		// (el log crudo já confirmó 1661 llamadas seguidas).
		static std::atomic<uint32_t> s_closeCount{ 0 };
		const bool wasOpen = camInstance->isOpen;
		camInstance->CloseCam();
		const uint32 count = s_closeCount.fetch_add(1, std::memory_order_relaxed) + 1;
		LogCamThrottled(fmt::format("[TwinpadCamera] CAMClose handle={} veces={} device_abierto={}",
			camHandle, count, wasOpen));
		return CAM_ERROR_SUCCESS;
	}

	sint32 CAMSubmitTargetSurface(sint32 camHandle, CAMTargetSurface* targetSurface)
	{
		CameraInstance* camInstance = GetCameraInstanceByHandle(camHandle);
		if (!camInstance)
		{
			cemuLog_log(LogType::Force, "[TwinpadCamera] CAMSubmitTargetSurface handle={} INVALIDO", camHandle);
			return CAM_ERROR_INVALID_HANDLE;
		}
		LogCamThrottled(fmt::format("[TwinpadCamera] CAMSubmitTargetSurface handle={} size={} ptr={:08X}",
			camHandle, (uint32)targetSurface->surfaceSize, (uint32)targetSurface->surfacePtr.GetMPTR()));
		camInstance->QueueTargetSurface(targetSurface);

		return CAM_ERROR_SUCCESS;
	}

	void reset()
	{
		g_cameraCounter = 0;
	}

	class : public COSModule
	{
		public:
		std::string_view GetName() override
		{
			return "camera";
		}

		void RPLMapped() override
		{
			cafeExportRegister("camera", CAMGetMemReq, LogType::Placeholder);
			cafeExportRegister("camera", CAMCheckMemSegmentation, LogType::Placeholder);
			cafeExportRegister("camera", CAMInit, LogType::Placeholder);
			cafeExportRegister("camera", CAMExit, LogType::Placeholder);
			cafeExportRegister("camera", CAMOpen, LogType::Placeholder);
			cafeExportRegister("camera", CAMClose, LogType::Placeholder);
			cafeExportRegister("camera", CAMSubmitTargetSurface, LogType::Placeholder);
		};

		void rpl_entry(uint32 moduleHandle, coreinit::RplEntryReason reason) override
		{
			if (reason == coreinit::RplEntryReason::Loaded)
			{
				reset();
			}
			else if (reason == coreinit::RplEntryReason::Unloaded)
			{
				// todo
			}
		}
	}s_COScameraModule;

	COSModule* GetModule()
	{
		return &s_COScameraModule;
	}
}

