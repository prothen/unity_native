# unity_native
This repository collects setup information about designing a native plugin for Unity3D with persistent objects.

## Plugin Setup (Native-Plugin / C++)
### VS2019
- Create new project select `C++` -> `Dynamic-Link-Library (DLL)`
- Set Project name (empty Location and solution name) & uncheck same directory for solution and directory
	- `Project` -> `Add new item` -> select Header file
- Select build for `x64` (Use configuration manager) and right click on solution and choose `Build Solution`
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

### Persistency of C++ Objects
With an exemplatory Test class, use `new` operator to dynamically allocate memory and store the reference in Unity3D as an `IntPtr`.


**Visual Studio:**
```cpp
Test* initialise() {
	Test* persistentObject = new Test();
	return persistentObject;
}

float update(Test* persistentObject) {
	persistentObject->increment();
	return persistentObject->get_value();
}
```


**Visual Studio:**
```csharp

public class NativePluginWrapper
{

    [DllImport("TemplateNativePlugin", EntryPoint = "initialise")]
    private static extern IntPtr initialise();

    [DllImport("TemplateNativePlugin", EntryPoint = "update")]
    private static extern float update(IntPtr objectReference);

    public IntPtr ObjectReference;

    public NativePluginWrapper()
    {
        ObjectReference = initialise();
    }

    public NativePluginWrapper(IntPtr objectReference)
    {
        ObjectReference = objectReference;
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
## FAQ
- [`EntryPointNotFoundException`](https://answers.unity.com/questions/1360004/entrypointnotfoundexception-native-plugin-help.html)
	- If entrypoint ommitted then the C# function name is used, the entrypoint parameter allows to use distinct c# function names
	- Choose Entrypoint as Functionname e.g. `[DllImport("__MyPlugin", EntryPoint = “displayNumber”)]` (C#) for `int displayNumber()`(C++)
	- [Official Microsoft DllImportAttribute.EntryPoint reference](https://docs.microsoft.com/en-us/dotnet/framework/interop/specifying-an-entry-point)
- [IntPtr namespace](https://docs.microsoft.com/en-us/dotnet/api/system.intptr?view=netcore-3.1)
	- requires `using System;` and initialise with `IntPtr.Zero;`

## References
- [C++ DLL Build VS2015/VS2019](https://docs.microsoft.com/en-us/cpp/build/walkthrough-creating-and-using-a-dynamic-link-library-cpp?view=vs-2019)
- [Common Language Infrastructure Interoperability](https://www.mono-project.com/docs/advanced/pinvoke/) (using other existing code)
- [Visual Studio Code - C++ Native Plugin Setup](https://www.alanzucconi.com/2015/10/11/how-to-write-native-plugins-for-unity/)
- [Native C++ Plugin persistent object reference in C#](https://answers.unity.com/questions/1200157/nonstatic-extern-functions-from-dll-plugin-import.html?_ga=2.245716996.1632391841.1599469631-970215712.1589606808)
- [C++ Google Style Guide](https://google.github.io/styleguide/cppguide.html)
- [C++ Background Thread in Native Plugin](https://ritchielozada.com/2017/06/16/interacting-with-plugins-in-unity-and-visual-studio/)