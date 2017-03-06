#pragma once

#include <QtCore/qglobal.h>

#ifndef BUILD_STATIC
# if defined(AEOPENGLVIEWER_LIB)
#  define AEOPENGLVIEWER_EXPORT Q_DECL_EXPORT
# else
#  define AEOPENGLVIEWER_EXPORT Q_DECL_IMPORT
# endif
#else
# define AEOPENGLVIEWER_EXPORT
#endif
