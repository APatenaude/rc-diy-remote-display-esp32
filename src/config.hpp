#pragma once

#define LGFX_USE_V1

#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{

    lgfx::Panel_ILI9488 _panel_instance;

    lgfx::Bus_SPI _bus_instance;

public:
    LGFX(void)
    {
        {
            auto cfg = _bus_instance.config();

            cfg.spi_host = VSPI_HOST;
            cfg.spi_mode = 0;
            cfg.freq_write = 40000000;
            cfg.freq_read = 16000000;
            cfg.spi_3wire = true;
            cfg.use_lock = true;
            cfg.dma_channel = SPI_DMA_CH_AUTO;
            cfg.pin_sclk = 4;
            cfg.pin_mosi = 23;
            cfg.pin_miso = -1;
            cfg.pin_dc = 2;

            _bus_instance.config(cfg);
            _panel_instance.setBus(&_bus_instance);
        }

        {
            auto cfg = _panel_instance.config();
            cfg.pin_cs = -1;
            cfg.pin_rst = 18;
            cfg.pin_busy = -1;

            cfg.panel_width = 320;
            cfg.panel_height = 480;
            cfg.offset_x = 0;
            cfg.offset_y = 0;
            cfg.offset_rotation = 0;
            cfg.dummy_read_pixel = 8;
            cfg.dummy_read_bits = 1;
            cfg.readable = true;
            cfg.invert = false;
            cfg.rgb_order = false;
            cfg.dlen_16bit = false;
            cfg.bus_shared = true;

            _panel_instance.config(cfg);
        }

        setPanel(&_panel_instance);
    }
};
