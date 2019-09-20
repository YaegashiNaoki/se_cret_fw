#!/bin/sh

nrfutil settings generate --family NRF52 \
    --application ./pca10040/s132/ses/Output/Release/Exe/ble_se_cret_s132.hex \
    --application-version 1 \
    --bootloader-version 1 \
    --bl-settings-version 1 \
    ./bin/settings.hex

mergehex -m ./bin/settings.hex ./pca10040/s132/ses/Output/Release/Exe/ble_se_cret_s132.hex -o ./bin/se_cret_app_settings_s132.hex

mergehex -m ./bin/s132_nrf52_5.0.0_softdevice.hex \
        ./bin/s132_nrf52_bootloader.hex \
        ./bin/se_cret_app_settings_s132.hex \
        -o ./bin/se_cret_all.hex
