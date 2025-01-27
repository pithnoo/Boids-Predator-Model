workspace "COMP3811-glcode"
	language "C++"
	cppdialect "C++20"

	platforms { "x64" }
	configurations { "debug", "release" }

	flags "NoPCH"
	flags "MultiProcessorCompile"

	startproject "exercise2"

	debugdir "%{wks.location}"
	objdir "_build_/%{cfg.buildcfg}-%{cfg.platform}-%{cfg.toolset}"
	targetsuffix "-%{cfg.buildcfg}-%{cfg.platform}-%{cfg.toolset}"
	
	-- Default toolset options
	filter "toolset:gcc or toolset:clang"
		linkoptions { "-pthread" }
		buildoptions { "-march=native", "-Wall", "-pthread" }

		-- Varriable-length arrays (VLAs) are an extension that GCC and clang
		-- have long supported. However, they are not part of the C++ standard.
		-- (MSVC will not compile code with VLAs.)
		buildoptions { "-Werror=vla" }

	filter "toolset:msc-*"
		warnings "extra" -- this enables /W4; default is /W3
		--buildoptions { "/W4" }
		buildoptions { "/utf-8" }
		buildoptions { "/permissive-" }
		defines { "_CRT_SECURE_NO_WARNINGS=1" }
		defines { "_SCL_SECURE_NO_WARNINGS=1" }

		buildoptions {
			"/wd4456", -- declaration of 'foo' hides previous local declaration
		}

	filter "*"

	-- default libraries
	filter "system:linux"
		links "dl"
	
	filter "system:windows"
		links "OpenGL32"

	filter "*"

	-- default outputs
	filter "kind:StaticLib"
		targetdir "lib/"

	filter "kind:ConsoleApp"
		targetdir "bin/"
		targetextension ".exe"
	
	filter "*"

	--configurations
	filter "debug"
		symbols "On"
		defines { "_DEBUG=1" }

	filter "release"
		optimize "On"
		defines { "NDEBUG=1" }

	filter "*"

-- Third party dependencies
include "third_party" 

-- Projects
project "exercise2"
	local sources = { 
		"exercise2/**.cpp",
		"exercise2/**.hpp",
		"exercise2/**.hxx",
		"exercise2/**.inl"
	}

	kind "ConsoleApp"
	location "exercise2"

	files( sources )

	dependson "exercise2-shaders"

	links "vmlib"
	links "support"

	links "x-stb"
	links "x-glad"
	links "x-glfw"

project "exercise2-shaders"
	local shaders = { 
		"assets/ex2/*.vert",
		"assets/ex2/*.frag",
		"assets/ex2/*.geom",
		"assets/ex2/*.tesc",
		"assets/ex2/*.tese",
		"assets/ex2/*.comp"
	}

	kind "Utility"
	location "assets/ex2"

	files( shaders )


project "support"
	local sources = { 
		"support/**.cpp",
		"support/**.hpp",
	}

	kind "StaticLib"
	location "support"

	files( sources )

	filter "*"

project "vmlib"
	local sources = { 
		"vmlib/**.cpp",
		"vmlib/**.hpp",
		"vmlib/**.hxx",
		"vmlib/**.inl"
	}

	kind "StaticLib"
	location "vmlib"

	files( sources )


--EOF
