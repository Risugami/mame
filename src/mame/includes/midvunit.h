// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/*************************************************************************

    Driver for Midway V-Unit games

**************************************************************************/

#include "audio/dcs.h"
#include "machine/adc0844.h"
#include "machine/ataintf.h"
#include "machine/midwayic.h"
#include "machine/timer.h"
#include "machine/watchdog.h"
#include "video/poly.h"
#include "emupal.h"
#include "screen.h"

#define MIDVUNIT_VIDEO_CLOCK    33000000

struct midvunit_object_data
{
	uint16_t *    destbase;
	uint8_t *     texbase;
	uint16_t      pixdata;
	uint8_t       dither;
};

class midvunit_device;

class midvunit_renderer : public poly_manager<float, midvunit_object_data, 2, 4000>
{
public:
	midvunit_renderer(midvunit_device &state);
	void process_dma_queue();
	void make_vertices_inclusive(vertex_t *vert);

private:
	midvunit_device &m_state;

	void render_flat(int32_t scanline, const extent_t &extent, const midvunit_object_data &extradata, int threadid);
	void render_tex(int32_t scanline, const extent_t &extent, const midvunit_object_data &extradata, int threadid);
	void render_textrans(int32_t scanline, const extent_t &extent, const midvunit_object_data &extradata, int threadid);
	void render_textransmask(int32_t scanline, const extent_t &extent, const midvunit_object_data &extradata, int threadid);
};

class midvunit_device : public device_t
{
public:
	midvunit_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
		: device_t(mconfig, type, tag, owner, clock),
		m_videoram(*this, "videoram", 32),
		m_textureram(*this, "textureram"),
		m_screen(*this, "screen"),
		m_nvram(*this, "nvram"),
		m_ram_base(*this, "ram_base"),
		m_fastram_base(*this, "fastram_base"),
		m_tms32031_control(*this, "32031_control"),
		m_midvplus_misc(*this, "midvplus_misc"),
		m_maincpu(*this, "maincpu"),
		m_watchdog(*this, "watchdog"),
		m_palette(*this, "palette"),
		m_adc(*this, "adc"),
		m_midway_serial_pic(*this, "serial_pic"),
		m_midway_serial_pic2(*this, "serial_pic2"),
		m_midway_ioasic(*this, "ioasic"),
		m_ata(*this, "ata"),
		m_timer(*this, "timer%u", 0U),
		m_dcs(*owner, "dcs"),
		m_generic_paletteram_32(*this, "paletteram"),
		m_optional_drivers(*this, "lamp%u", 0U),
		m_dsw(*owner, "DSW"),
		m_in1(*this, "IN1"),
		m_motion(*this, "MOTION") { }

	uint16_t m_page_control;
	uint16_t m_dma_data[16];
	uint8_t m_video_changed;

	required_shared_ptr<uint16_t> m_videoram;
	required_shared_ptr<uint32_t> m_textureram;
	required_device<screen_device> m_screen;

	DECLARE_CUSTOM_INPUT_MEMBER(motion_r);

	void midvcommon(machine_config &config);
	void init_crusnusa(offs_t speedup);
	void init_crusnwld(offs_t speedup);
	void init_offroadc();
	void init_wargods();
	void video_start();
	void set_link(uint8_t player_id, uint8_t player_count, midvunit_device *target);

protected:
	enum
	{
		TIMER_SCANLINE
	};

	optional_shared_ptr<uint32_t> m_nvram;
	required_shared_ptr<uint32_t> m_ram_base;
	optional_shared_ptr<uint32_t> m_fastram_base;
	required_shared_ptr<uint32_t> m_tms32031_control;
	optional_shared_ptr<uint32_t> m_midvplus_misc;

	uint8_t m_cmos_protected;
	uint16_t m_control_data;
	uint8_t m_adc_shift;
	uint16_t m_last_port0;
	uint8_t m_shifter_state;
	double m_timer_rate;
	uint16_t m_bit_index;
	int m_lastval;
	uint32_t *m_generic_speedup;
	uint16_t m_video_regs[16];
	uint8_t m_dma_data_index;
	emu_timer *m_scanline_timer;
	std::unique_ptr<midvunit_renderer> m_poly;
	uint8_t m_galil_input_index;
	uint8_t m_galil_input_length;
	const char *m_galil_input;
	uint8_t m_galil_output_index;
	char m_galil_output[450];
	uint8_t m_wheel_board_output;
	uint8_t m_link_irq;
	uint32_t m_wheel_board_last;
	uint32_t m_wheel_board_u8_latch;
	uint16_t m_link_dips;
	uint16_t m_link_data;
	uint16_t m_link_input_last;
	uint16_t m_link_output_last;
	midvunit_device *m_link_device;
	DECLARE_WRITE32_MEMBER(midvunit_dma_queue_w);
	DECLARE_READ32_MEMBER(midvunit_dma_queue_entries_r);
	DECLARE_READ32_MEMBER(midvunit_dma_trigger_r);
	DECLARE_WRITE32_MEMBER(midvunit_page_control_w);
	DECLARE_READ32_MEMBER(midvunit_page_control_r);
	DECLARE_WRITE32_MEMBER(midvunit_video_control_w);
	DECLARE_READ32_MEMBER(midvunit_scanline_r);
	DECLARE_WRITE32_MEMBER(midvunit_videoram_w);
	DECLARE_READ32_MEMBER(midvunit_videoram_r);
	DECLARE_WRITE32_MEMBER(midvunit_paletteram_w);
	DECLARE_WRITE32_MEMBER(midvunit_textureram_w);
	DECLARE_READ32_MEMBER(midvunit_textureram_r);
	DECLARE_READ32_MEMBER(port0_r);
	DECLARE_READ32_MEMBER(adc_r);
	DECLARE_WRITE32_MEMBER(adc_w);
	DECLARE_WRITE32_MEMBER(midvunit_cmos_protect_w);
	DECLARE_WRITE32_MEMBER(midvunit_cmos_w);
	DECLARE_READ32_MEMBER(midvunit_cmos_r);
	DECLARE_WRITE32_MEMBER(midvunit_control_w);
	DECLARE_WRITE32_MEMBER(crusnwld_control_w);
	DECLARE_WRITE32_MEMBER(midvunit_sound_w);
	DECLARE_READ32_MEMBER(tms32031_control_r);
	DECLARE_WRITE32_MEMBER(tms32031_control_w);
	DECLARE_READ32_MEMBER(crusnwld_serial_status_r);
	DECLARE_READ32_MEMBER(crusnwld_serial_data_r);
	DECLARE_WRITE32_MEMBER(crusnwld_serial_data_w);
	DECLARE_READ32_MEMBER(bit_data_r);
	DECLARE_WRITE32_MEMBER(bit_reset_w);
	DECLARE_READ32_MEMBER(offroadc_serial_status_r);
	DECLARE_READ32_MEMBER(offroadc_serial_data_r);
	DECLARE_WRITE32_MEMBER(offroadc_serial_data_w);
	DECLARE_READ32_MEMBER(midvplus_misc_r);
	DECLARE_WRITE32_MEMBER(midvplus_misc_w);
	DECLARE_WRITE8_MEMBER(midvplus_xf1_w);
	DECLARE_READ32_MEMBER(generic_speedup_r);
	DECLARE_READ32_MEMBER(midvunit_wheel_board_r);
	DECLARE_WRITE32_MEMBER(midvunit_wheel_board_w);
	DECLARE_READ16_MEMBER(midvunit_dipswitches_r);
	DECLARE_READ32_MEMBER(midvunit_irq_r);
	DECLARE_READ32_MEMBER(midvunit_comm_r);
	DECLARE_WRITE32_MEMBER(midvunit_comm_w);
	void set_input(const char *s);
	virtual void device_start() override;
	virtual void device_reset() override;
	DECLARE_MACHINE_RESET(midvplus);
	uint32_t screen_update_midvunit(screen_device &screen, bitmap_ind16 &bitmap, const rectangle &cliprect);
	TIMER_CALLBACK_MEMBER(scanline_timer_cb);
	required_device<tms32031_device> m_maincpu;
	required_device<watchdog_timer_device> m_watchdog;
	required_device<palette_device> m_palette;
	optional_device<adc0844_device> m_adc;
	optional_device<midway_serial_pic_device> m_midway_serial_pic;
	optional_device<midway_serial_pic2_device> m_midway_serial_pic2;
	optional_device<midway_ioasic_device> m_midway_ioasic;
	optional_device<ata_interface_device> m_ata;
	required_device_array<timer_device, 2> m_timer;
	optional_device<dcs_audio_device> m_dcs;
	required_shared_ptr<uint32_t> m_generic_paletteram_32;
	output_finder<8> m_optional_drivers;
	optional_ioport m_dsw;
	optional_ioport m_in1;
	optional_ioport m_motion;
	void postload();

	void midvunit_map(address_map &map);
	void midvplus_map(address_map &map);

	virtual void device_timer(emu_timer &timer, device_timer_id id, int param, void *ptr) override;
};

class midvunit_state : public driver_device
{
public:
	midvunit_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag),
		m_player(*this, "player%u", 0U) { }

	void common(machine_config &config);
	void crusnusa41(machine_config &config);
	void crusnusa40(machine_config &config);
	void crusnusa21(machine_config &config);
	void crusnwld25(machine_config &config);
	void crusnwld24(machine_config &config);
	void crusnwld23(machine_config &config);
	void crusnwld20(machine_config &config);
	void crusnwld19(machine_config &config);
	void crusnwld17(machine_config &config);
	void crusnwld13(machine_config &config);

	void init_crusnu41();
	void init_crusnu40();
	void init_crusnu21();
	void init_crusnwld();
	void init_offroadc();
	void init_wargods();
	uint8_t player_count() { return 2; }

private:
	virtual void video_start() override;
	optional_device_array<midvunit_device, 4> m_player;
};

class crusnusa_device : public midvunit_device
{
public:
	crusnusa_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
		: midvunit_device(mconfig, type, tag, owner, clock) { }
		
	virtual ioport_constructor device_input_ports() const override;
private:
	virtual void device_add_mconfig(machine_config &config) override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNUSA41, crusnusa41_device)


class crusnusa41_device : public crusnusa_device
{
public:
	crusnusa41_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnusa_device(mconfig, MIDVUNIT_CRUSNUSA41, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNUSA40, crusnusa40_device)

class crusnusa40_device : public crusnusa_device
{
public:
	crusnusa40_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnusa_device(mconfig, MIDVUNIT_CRUSNUSA40, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNUSA21, crusnusa21_device)

class crusnusa21_device : public crusnusa_device
{
public:
	crusnusa21_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnusa_device(mconfig, MIDVUNIT_CRUSNUSA21, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

class crusnwld_device : public midvunit_device
{
public:
	crusnwld_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
		: midvunit_device(mconfig, type, tag, owner, clock) { }
		
	virtual ioport_constructor device_input_ports() const override;
private:
	virtual void device_add_mconfig(machine_config &config) override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNWRD25, crusnwld25_device)

class crusnwld25_device : public crusnwld_device
{
public:
	crusnwld25_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnwld_device(mconfig, MIDVUNIT_CRUSNWRD25, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNWRD24, crusnwld24_device)

class crusnwld24_device : public crusnwld_device
{
public:
	crusnwld24_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnwld_device(mconfig, MIDVUNIT_CRUSNWRD24, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNWRD23, crusnwld23_device)

class crusnwld23_device : public crusnwld_device
{
public:
	crusnwld23_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnwld_device(mconfig, MIDVUNIT_CRUSNWRD23, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNWRD20, crusnwld20_device)

class crusnwld20_device : public crusnwld_device
{
public:
	crusnwld20_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnwld_device(mconfig, MIDVUNIT_CRUSNWRD20, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNWRD19, crusnwld19_device)

class crusnwld19_device : public crusnwld_device
{
public:
	crusnwld19_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnwld_device(mconfig, MIDVUNIT_CRUSNWRD19, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNWRD17, crusnwld17_device)

class crusnwld17_device : public crusnwld_device
{
public:
	crusnwld17_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnwld_device(mconfig, MIDVUNIT_CRUSNWRD17, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};

DECLARE_DEVICE_TYPE(MIDVUNIT_CRUSNWRD13, crusnwld13_device)

class crusnwld13_device : public crusnwld_device
{
public:
	crusnwld13_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: crusnwld_device(mconfig, MIDVUNIT_CRUSNWRD13, tag, owner, clock) { }

private:
	const tiny_rom_entry *device_rom_region() const override;
};
