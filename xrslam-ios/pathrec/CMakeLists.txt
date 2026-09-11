cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(xrslam-ios-pathrec LANGUAGES CXX Swift)

set(CODE_SIGN_IDENTITY "iPhone Developer")      # /usr/bin/env xcrun security find-identity -v -p codesigning
set(DEVELOPMENT_TEAM "${IOS_DEVELOPMENT_TEAM}") # XCode: Build Settings > Signing > Development Team

set(TARGETED_DEVICE_FAMILY "1") # 1: iphone; 2: ipad; 1,2: both

# PathRec: minimal walk-path recorder on top of the XRSLAM (RD-VIO) engine.
set(EXECUTABLE_NAME xrslam-ios-pathrec)
set(BUNDLE_NAME "PathRec")
set(BUNDLE_IDENTIFIER "${APP_IDENTIFIER_PREFIX}.pathrec")
set(BUNDLE_VERSION "0.1")
set(LAUNCH_STORYBOARD LaunchScreen)
set(SWIFT_OBJC_BRIDGING_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/src/SwiftBridge.h)

set(INFO_PLIST_TEMPLATE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Info.plist.in)
include_directories(${CMAKE_SOURCE_DIR}/include/)

set(XRSLAM_IOS_PATHREC_SOURCES
    ${CMAKE_SOURCE_DIR}/include/XRSLAM.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/XRSLAM_iOS.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/XRSLAM_iOS.mm
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Camera.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Motion.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/UIDevice+Model.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/AppDelegate.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PathRecViewController.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/PathView.swift
    ${CMAKE_CURRENT_SOURCE_DIR}/src/loopclosure/pose_graph.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/loopclosure/pose_graph.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/loopclosure/loop_closure.h
    ${CMAKE_CURRENT_SOURCE_DIR}/src/loopclosure/loop_closure.cpp
)

set(XRSLAM_IOS_PATHREC_STORYBOARDS
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/${LAUNCH_STORYBOARD}.storyboard
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/icon.png
)

set(XRSLAM_IOS_PATHREC_RESOURCES
    ${XRSLAM_IOS_PATHREC_STORYBOARDS}
    ${CMAKE_CURRENT_SOURCE_DIR}/resources/Assets.xcassets
    ${CMAKE_CURRENT_SOURCE_DIR}/configs
)

set(MACOSX_BUNDLE_ICON_FILE icon.png)

add_executable(${EXECUTABLE_NAME}
    MACOSX_BUNDLE
    ${SWIFT_OBJC_BRIDGING_HEADER}
    ${XRSLAM_IOS_PATHREC_SOURCES}
    ${XRSLAM_IOS_PATHREC_STORYBOARDS}
    ${XRSLAM_IOS_PATHREC_RESOURCES}
)

target_link_libraries(${EXECUTABLE_NAME}
    PRIVATE
        xrslam
)

set_target_properties(${EXECUTABLE_NAME} PROPERTIES
    MACOSX_BUNDLE_INFO_PLIST ${INFO_PLIST_TEMPLATE}
    RESOURCE "${XRSLAM_IOS_PATHREC_RESOURCES}"
)

set_xcode_property(${EXECUTABLE_NAME} CODE_SIGN_IDENTITY ${CODE_SIGN_IDENTITY} "All")
set_xcode_property(${EXECUTABLE_NAME} DEVELOPMENT_TEAM ${DEVELOPMENT_TEAM} "All")
set_xcode_property(${EXECUTABLE_NAME} TARGETED_DEVICE_FAMILY ${TARGETED_DEVICE_FAMILY} "All")
set_xcode_property(${EXECUTABLE_NAME} IPHONEOS_DEPLOYMENT_TARGET "15.0" "All")
set_xcode_property(${EXECUTABLE_NAME} SWIFT_OBJC_BRIDGING_HEADER ${SWIFT_OBJC_BRIDGING_HEADER} "All")
set_xcode_property(${EXECUTABLE_NAME} SWIFT_VERSION "5.0" "All")
set_xcode_property(${EXECUTABLE_NAME} CLANG_CXX_LANGUAGE_STANDARD "c++17" "All")
set_xcode_property(${EXECUTABLE_NAME} DEBUG_INFORMATION_FORMAT "dwarf-with-dsym" "All")
set_xcode_property(${EXECUTABLE_NAME} LD_RUNPATH_SEARCH_PATHS "@executable_path/Frameworks" "All")
if(ENABLE_BITCODE)
    set_xcode_property(${EXECUTABLE_NAME} ENABLE_BITCODE "YES" "All")
else()
    set_xcode_property(${EXECUTABLE_NAME} ENABLE_BITCODE "NO" "All")
endif()
if(ENABLE_ARC)
    set_xcode_property(${EXECUTABLE_NAME} CLANG_ENABLE_OBJC_ARC "YES" "All")
else()
    set_xcode_property(${EXECUTABLE_NAME} CLANG_ENABLE_OBJC_ARC "NO" "All")
endif()
