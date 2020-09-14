// TemplateNativePlugin.cpp : Defines the exported functions for the DLL.
//

#include "pch.h"
#include "framework.h"
#include "TemplateNativePlugin.h"
//
#include <stdio.h>
#include <conio.h>
#include <Python.h>

Test* initialise() {
	Test* persistentObject = new Test();
	return persistentObject;
}

float update(Test* persistentObject) {
	persistentObject->increment();
	return persistentObject->get_value();
}

int launch() {
		// c:\\users\\colma\\hcps\\Interfaces\\Python\\
		
		char filename[] = "client.py";
		FILE* fp;

		Py_Initialize();

		fp = _Py_fopen(filename, "r");
		PyRun_SimpleFile(fp, filename);

		Py_Finalize();
		return 0;
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
