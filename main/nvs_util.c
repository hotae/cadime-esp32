#include "nvs_util.h"

void cd_nvs_write_panid(uint16_t _pan_id)
{
    esp_err_t err;

    // Open
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("cadime", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // Write
        printf("Updating pan_id in NVS ... ");
        err = nvs_set_u16(my_handle, "cadime_panid", _pan_id);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        printf("Committing updates in NVS ... ");
        err = nvs_commit(my_handle);
        printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

        // Close
        nvs_close(my_handle);
    }
}

uint16_t cd_nvs_read_panid()
{
    esp_err_t err;
    uint16_t pan_id = 0;

    // Open
    printf("Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle_t my_handle;
    err = nvs_open("cadime", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
    } else {
        // Read
        printf("Reading restart counter from NVS ... ");
        err = nvs_get_u16(my_handle, "cadime_panid", &pan_id);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
                printf("PAN_ID = %04x\n", pan_id);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
        // Close
        nvs_close(my_handle);
    }

    return pan_id;
}
