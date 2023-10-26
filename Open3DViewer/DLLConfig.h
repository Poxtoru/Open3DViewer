#pragma once
#ifdef VIEWER_DLL
#define DLLExport __declspec(dllimport)
#else
#define DLLExport __declspec(dllexport)
#endif