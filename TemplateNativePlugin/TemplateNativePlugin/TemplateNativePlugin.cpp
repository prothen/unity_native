// TemplateNativePlugin.cpp : Defines the exported functions for the DLL.
//

#include "pch.h"
#include "framework.h"
#include "TemplateNativePlugin.h"


Test* initialise() {
	Test persistentObject = Test();
	return &persistentObject;
}

float update(Test* persistentObject) {
	persistentObject->increment();
	return persistentObject->get_value();
}

// This is an example of an exported variable
TEMPLATENATIVEPLUGIN_API int nTemplateNativePlugin = 0;

// This is an example of an exported function.
TEMPLATENATIVEPLUGIN_API int fnTemplateNativePlugin(void)
{
	return 0;
}

// This is the constructor of a class that has been exported.
CTemplateNativePlugin::CTemplateNativePlugin()
{
    return;
}
