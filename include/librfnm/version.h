#pragma once

// Single source of truth for the librfnm version. CMake parses the three numeric
// components below (see CMakeLists.txt); keep each on its own line, numeric only.
// RFNM_VERSION carries the human-facing form including any pre-release suffix;
// machine-compared values (CMake, pkg-config, SONAME) use only the numeric triple.
#define RFNM_VERSION_MAJOR 0
#define RFNM_VERSION_MINOR 2
#define RFNM_VERSION_PATCH 2
#define RFNM_VERSION_SUFFIX "-beta"

#define RFNM_VERSION_STR2(x) #x
#define RFNM_VERSION_STR(x) RFNM_VERSION_STR2(x)
#define RFNM_VERSION \
	RFNM_VERSION_STR(RFNM_VERSION_MAJOR) "." \
	RFNM_VERSION_STR(RFNM_VERSION_MINOR) "." \
	RFNM_VERSION_STR(RFNM_VERSION_PATCH) RFNM_VERSION_SUFFIX
