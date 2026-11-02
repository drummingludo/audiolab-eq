cmake_minimum_required(VERSION 3.22)

# Mac : un seul binaire pour Apple Silicon et Intel
set(CMAKE_OSX_ARCHITECTURES "arm64;x86_64" CACHE STRING "")
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.13" CACHE STRING "")

project(AudioLabEQ VERSION 1.3.0 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# JUCE 8 (licence AGPLv3 : le code de ce plugin est public)
set(ALEQ_JUCE_PATH "" CACHE PATH "Dossier JUCE local (sinon téléchargé)")
if(ALEQ_JUCE_PATH)
    add_subdirectory(${ALEQ_JUCE_PATH} JUCE)
else()
    include(FetchContent)
    FetchContent_Declare(JUCE
        GIT_REPOSITORY https://github.com/juce-framework/JUCE.git
        GIT_TAG 8.0.15
        GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(JUCE)
endif()

set(ALEQ_FORMATS VST3)
if(APPLE)
    list(APPEND ALEQ_FORMATS AU)
endif()
if(ALEQ_STANDALONE)
    list(APPEND ALEQ_FORMATS Standalone)
endif()

juce_add_plugin(AudioLabEQ
    PRODUCT_NAME "Audio Lab EQ"
    PLUGIN_NAME "Lab EQ"
    DESCRIPTION "24-band parametric equaliser from Audio Lab (Chimiste du son)"
    COMPANY_NAME "Transversal Studio"
    COMPANY_WEBSITE "https://transversalstudio.fr"
    COMPANY_EMAIL "ludo@transversalstudio.fr"
    BUNDLE_ID "fr.transversalstudio.audiolab.eq"
    PLUGIN_MANUFACTURER_CODE Trsv
    PLUGIN_CODE AlEq
    FORMATS ${ALEQ_FORMATS}
    IS_SYNTH FALSE
    NEEDS_MIDI_INPUT FALSE
    NEEDS_MIDI_OUTPUT FALSE
    IS_MIDI_EFFECT FALSE
    EDITOR_WANTS_KEYBOARD_FOCUS TRUE
    VST3_CATEGORIES Fx EQ
    AU_MAIN_TYPE kAudioUnitType_Effect
    COPY_PLUGIN_AFTER_BUILD FALSE)

juce_add_binary_data(ALEQData SOURCES
    Resources/Poppins-Regular.ttf
    Resources/Poppins-Medium.ttf
    Resources/Poppins-SemiBold.ttf
    Resources/Poppins-Bold.ttf)

target_sources(AudioLabEQ PRIVATE
    Source/PluginProcessor.cpp
    Source/PluginEditor.cpp
    Source/EqDisplay.cpp)

target_include_directories(AudioLabEQ PRIVATE Source)
if(MSVC)
    target_compile_options(AudioLabEQ PRIVATE /utf-8)
endif()

target_compile_definitions(AudioLabEQ PUBLIC
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    JUCE_VST3_CAN_REPLACE_VST2=0
    JUCE_REPORT_APP_USAGE=0
    JUCE_MODAL_LOOPS_PERMITTED=0)

target_link_libraries(AudioLabEQ
    PRIVATE
        ALEQData
        juce::juce_audio_utils
        juce::juce_audio_processors
        juce::juce_dsp
        juce::juce_gui_basics
    PUBLIC
        juce::juce_recommended_config_flags
        juce::juce_recommended_lto_flags
        juce::juce_recommended_warning_flags)

# Outils de test (DSP, captures de l'interface hors DAW) — non livrés
option(ALEQ_TESTS "Construire les outils de test" OFF)
if(ALEQ_TESTS)
    juce_add_console_app(ALEQDspTest PRODUCT_NAME "ALEQDspTest")
    target_sources(ALEQDspTest PRIVATE tests/dsp_test.cpp)
    target_include_directories(ALEQDspTest PRIVATE Source)
    target_compile_definitions(ALEQDspTest PRIVATE JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0)
    target_link_libraries(ALEQDspTest PRIVATE juce::juce_dsp juce::juce_audio_basics juce::juce_recommended_config_flags)

    juce_add_console_app(ALEQRenderUI PRODUCT_NAME "ALEQRenderUI")
    target_sources(ALEQRenderUI PRIVATE tests/render_ui.cpp)
    target_include_directories(ALEQRenderUI PRIVATE Source)
    target_compile_definitions(ALEQRenderUI PRIVATE JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0)
    target_link_libraries(ALEQRenderUI PRIVATE AudioLabEQ ALEQData juce::juce_audio_utils juce::juce_dsp juce::juce_gui_basics juce::juce_recommended_config_flags)

    juce_add_console_app(ALEQProcessorTest PRODUCT_NAME "ALEQProcessorTest")
    target_sources(ALEQProcessorTest PRIVATE tests/processor_test.cpp)
    target_include_directories(ALEQProcessorTest PRIVATE Source)
    target_compile_definitions(ALEQProcessorTest PRIVATE JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0)
    target_link_libraries(ALEQProcessorTest PRIVATE AudioLabEQ ALEQData juce::juce_audio_utils juce::juce_dsp juce::juce_gui_basics juce::juce_recommended_config_flags)
endif()
