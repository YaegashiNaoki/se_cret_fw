#!/bin/sh

nrfutil pkg generate \
    --hw-version 52 \
    --application-version 2 \
    --application ../pca10040/s132/ses/Output/Release/Exe/ble_app_ancs_c_pca10040_s132.hex \
    --sd-req 0x9D \
    --key-file private.pem \
    se_cret_dfu_pkg.zip
