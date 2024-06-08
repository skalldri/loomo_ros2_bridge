plugins {
    alias(libs.plugins.android.application)
    alias(libs.plugins.jetbrains.kotlin.android)
}

android {
    namespace = "com.autom8ed.lr2"
    compileSdk = 34

    defaultConfig {
        applicationId = "com.autom8ed.lr2"
        minSdk = 22
        targetSdk = 34
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        vectorDrawables {
            useSupportLibrary = true
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
        jvmTarget = "1.8"
    }
    buildFeatures {
        compose = true
    }
    composeOptions {
        kotlinCompilerExtensionVersion = "1.5.1"
    }
    packaging {
        resources {
            excludes += "/META-INF/{AL2.0,LGPL2.1}"
        }
    }
}

dependencies {
    // Logging framework, used by ROS2 and the Segway modules
    implementation(libs.slf4j)
    implementation(libs.slf4j.android)

    // BEGIN SEGWAY LIBRARIES

    implementation(libs.segway.robot.visionsdk)
    // Dupe of visionsdk, must have been renamed at some point. Lots of duplicate functions if this gets included
    // implementation(libs.segway.robot.vision)
    implementation(libs.segway.robot.speech.sdk)
    implementation(libs.segway.robot.headsdk)
    implementation(libs.segway.robot.basesdk)
    implementation(libs.segway.robot.sensorsdk)
    implementation(libs.segway.robot.support.lib)
    implementation(libs.segway.robot.basicclass)
    implementation(libs.segway.robot.sdkbase)
    /*
    // Not worth the pain. These cause issues with duplicate inclusion of "org.slf4j", and it seems like
    // Kotlin's Gradle DSL doesn't provide good ways to avoid the conflict.
    // Just comment these out, we don't need them / care about them.
    implementation(libs.segway.robot.base.connectivity.sdk) {
        exclude(group = "org.slf4j")
    }
    implementation(libs.segway.robot.robot.connectivity.sdk) {
        exclude(group = "org.slf4j")
    }
    implementation(libs.segway.robot.mobile.connectivity.sdk) {
        exclude(group = "org.slf4j")
    }
    */

    // END SEGWAY LIBRARIES

    // ROS2, pull it from the AAR file
    implementation(group = "", name = "rclandroid-release", ext = "aar") {
        exclude(group = "org.slf4j")
    }

    implementation(libs.androidx.core.ktx)
    implementation(libs.androidx.lifecycle.runtime.ktx)
    implementation(libs.androidx.activity.compose)
    implementation(platform(libs.androidx.compose.bom))
    implementation(libs.androidx.ui)
    implementation(libs.androidx.ui.graphics)
    implementation(libs.androidx.ui.tooling.preview)
    implementation(libs.androidx.material3)


    testImplementation(libs.junit)
    androidTestImplementation(libs.androidx.junit)
    androidTestImplementation(libs.androidx.espresso.core)
    androidTestImplementation(platform(libs.androidx.compose.bom))
    androidTestImplementation(libs.androidx.ui.test.junit4)
    debugImplementation(libs.androidx.ui.tooling)
    debugImplementation(libs.androidx.ui.test.manifest)
}