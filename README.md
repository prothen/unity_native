# unity_native
This repository collects setup information about designing a native plugin for Unity3D with persistent objects.

## Plugin Setup (Native-Plugin / C++)
### VS2019
- Create new project select `C++` -> `Dynamic-Link-Library (DLL)`
- Set Project name (empty Location and sultion name) & uncheck same directory for solution and directory
	- `Project` -> `Add new item` -> select Header file
- Write C++ code and expose simple C interface to Unity3D through preprocessor `__declspec` 
**C++ Code Sample**:
```cpp
// header.h
#pragma once

#ifdef MATHLIBRARY_EXPORTS
#define MATHLIBRARY_API __declspec(dllexport)
#else
#define MATHLIBRARY_API __declspec(dllimport)
#endif
```
this exports as prefix all functions such as
```cpp
extern "C" MATHLIBRARY_API bool fibonacci_next();
```
- Then copy paste the generate dynamic linked library (dll) into the project directory `Assets/Plugins` and access with following C# code
	- Note: Mainly two elements are necessary
		1. Ensure the `using System.Runtime.InteropServices;`
		2. Before using the C function import the Dll with `[DllImport ("PluginName")]`

**C# Code Sample**:
```csharp
    using UnityEngine;
    using System.Runtime.InteropServices;

    class SomeScript : MonoBehaviour {

       #if UNITY_IPHONE
   
       // On iOS plugins are statically linked into
       // the executable, so we have to use __Internal as the
       // library name.
       [DllImport ("__Internal")]

       #else

       // Other platforms load plugins dynamically, so pass the name
       // of the plugin's dynamic library.
       [DllImport ("PluginName")]
    
       #endif

       private static extern float FooPluginFunction ();

       void Awake () {
          // Calls the FooPluginFunction inside the plugin
          // And prints 5 to the console
          print (FooPluginFunction ());
       }
    }
```

### Callbacks to GraphicsEvents
- see headers under `C:\Program Files\Unity\Hub\Editor\2019.4.0f1\Editor\Data\PluginAPI` (if using Docker Hub)
```cpp
#include "IUnityInterface.h"
#include "IUnityGraphics.h"
    
static IUnityInterfaces* s_UnityInterfaces = NULL;
static IUnityGraphics* s_Graphics = NULL;
static UnityGfxRenderer s_RendererType = kUnityGfxRendererNull;
    
// Unity plugin load event
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
    UnityPluginLoad(IUnityInterfaces* unityInterfaces)
{
    s_UnityInterfaces = unityInterfaces;
    s_Graphics = unityInterfaces->Get<IUnityGraphics>();
        
    s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent);
        
    // Run OnGraphicsDeviceEvent(initialize) manually on plugin load
    // to not miss the event in case the graphics device is already initialized
    OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize);
}
    
// Unity plugin unload event
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
    UnityPluginUnload()
{
    s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
}
    
static void UNITY_INTERFACE_API
    OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType)
{
    switch (eventType)
    {
        case kUnityGfxDeviceEventInitialize:
        {
            s_RendererType = s_Graphics->GetRenderer();
            //TODO: user initialization code
            break;
        }
        case kUnityGfxDeviceEventShutdown:
        {
            s_RendererType = kUnityGfxRendererNull;
            //TODO: user shutdown code
            break;
        }
        case kUnityGfxDeviceEventBeforeReset:
        {
            //TODO: user Direct3D 9 code
            break;
        }
        case kUnityGfxDeviceEventAfterReset:
        {
            //TODO: user Direct3D 9 code
            break;
        }
    };
}
```


## References
- [C++ DLL Build VS2015/VS2019](https://docs.microsoft.com/en-us/cpp/build/walkthrough-creating-and-using-a-dynamic-link-library-cpp?view=vs-2019)
- [Common Language Infrastructure Interoperability](https://www.mono-project.com/docs/advanced/pinvoke/) (using other existing code)