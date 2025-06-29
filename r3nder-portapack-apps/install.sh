#!/bin/bash
set -e

echo "[*] Installing R3NDER OPS custom apps..."

TARGET_DIR=firmware/apps
MENU_FILE=firmware/menu_r3nder.cpp

mkdir -p "$TARGET_DIR"

app_files=(
    app_r3nder_rfbeacon.cpp
    app_r3nder_rfjukebox.cpp
    app_r3nder_triggerradar.cpp
    app_r3nder_burstjammer.cpp
    app_r3nder_gpsfuzzer.cpp
    app_r3nder_microwavepresence.cpp
    app_r3nder_rfidnoise.cpp
    app_r3nder_rfmirror.cpp
    app_r3nder_signalviewer.cpp
)

for file in "${app_files[@]}"; do
    cp "apps/$file" "$TARGET_DIR/"
    echo "Copied $file to $TARGET_DIR"
done

if [ ! -f "$MENU_FILE" ]; then
    {
        echo "// Auto-generated R3NDER OPS menu"
        for file in "${app_files[@]}"; do
            echo "#include \"$file\""
        done
        echo
        echo 'Menu r3nder_menu("R3NDER OPS");'
        class_names=(
            App_R3nderRFBeacon
            App_R3nderRFJukebox
            App_R3nderTriggerRadar
            App_R3nderBurstJammer
            App_R3nderGPSFuzzer
            App_R3nderMicrowavePresence
            App_R3nderRFIDNoise
            App_R3nderRFMirror
            App_R3nderSignalViewer
        )
        for class in "${class_names[@]}"; do
            echo "r3nder_menu.add(new ${class}());"
        done
        echo 'main_menu.add(&r3nder_menu);'
    } > "$MENU_FILE"
    echo "Created $MENU_FILE"
fi

echo "[+] Done. Now rebuild your firmware with 'make -j4' from the firmware root." 
