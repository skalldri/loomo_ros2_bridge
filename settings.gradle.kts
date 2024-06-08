pluginManagement {
    repositories {
        google {
            content {
                includeGroupByRegex("com\\.android.*")
                includeGroupByRegex("com\\.google.*")
                includeGroupByRegex("androidx.*")
            }
        }
        mavenCentral()
        gradlePluginPortal()
    }
}
dependencyResolutionManagement {
    repositories {
        google()
        mavenCentral()
        maven { url = uri("${rootDir}/deps/localmaven") }
        flatDir {
            name = "rclandroid-folder"
            dirs("${rootDir}/deps/rclandroid")
        }
    }
}

rootProject.name = "Loomo ROS2 Bridge"
include(":app")
 