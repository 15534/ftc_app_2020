# Add project specific ProGuard rules here.
# You can control the set of applied configuration files using the
# proguardFiles setting in build.gradle.
#
# For more details, see
#   http://developer.android.com/guide/developing/tools/proguard.html

# If your project uses WebView with JS, uncomment the following
# and specify the fully qualified class name to the JavaScript interface
# class:
#-keepclassmembers class fqcn.of.javascript.interface.for.webview {
#   public *;
#}

# Uncomment this to preserve the line number information for
# debugging stack traces.
#-keepattributes SourceFile,LineNumberTable

# If you keep the line number information, uncomment this to
# hide the original source file name.
#-renamesourcefileattribute SourceFile

# Avoid touching the FTC SDK
-keep class com.qualcomm.** {*;}
-keep class org.firstinspires.** {*;}
-keep class com.google.** {*;}
-keep class com.vuforia.** {*;}
-keep class org.tensorflow.** {*;}
-keep class javax.** {*;} # this is apparently required

-dontwarn com.qualcomm.**
-dontwarn org.firstinspires.**
-dontwarn com.vuforia.**
-dontwarn com.sun.**
-dontwarn org.tensorflow.**

# Op modes
-keep public class * extends com.qualcomm.robotcore.eventloop.opmode.OpMode

# Kotlin
-dontwarn kotlin.**

# ACME libs
# this keep is actually required for serialization
-keep class com.acmerobotics.** {*;}
-dontwarn com.acmerobotics.**

# RE2
-keep class org.openftc.** {*;}
-dontwarn org.openftc.**

# Other deps
-dontwarn com.fasterxml.**
-dontwarn org.yaml.**
-dontwarn org.apache.**
-dontwarn com.google.gson.**

# Misc
-dontnote **
