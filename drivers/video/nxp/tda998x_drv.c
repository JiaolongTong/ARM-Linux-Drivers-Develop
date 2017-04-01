/*
 * Copyright (C) 2012 Texas Instruments
 * Authors:
 *	Rob Clark <robdclark@gmail.com>
 *	Arun Joseph <arunjoseph@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */



#include <linux/module.h>

#include <linux/hdmi.h>
#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>


#define DBG(fmt, ...) DRM_DEBUG(fmt"\n", ##__VA_ARGS__)

struct tda998x_priv {
	struct i2c_client *cec;
	uint16_t rev;
	uint8_t current_page;
	int dpms;
};

/* The TDA9988 series of devices use a paged register scheme.. to simplify
 * things we encode the page # in upper bits of the register #.  To read/
 * write a given register, we need to make sure CURPAGE register is set
 * appropriately.  Which implies reads/writes are not atomic.  Fun!
 */

#define REG(page, addr) (((page) << 8) | (addr))
#define REG2ADDR(reg)   ((reg) & 0xff)
#define REG2PAGE(reg)   (((reg) >> 8) & 0xff)

#define REG_CURPAGE               0xff                /* write */


/* Page 00h: General Control */
#define REG_VERSION_LSB           REG(0x00, 0x00)     /* read */
#define REG_MAIN_CNTRL0           REG(0x00, 0x01)     /* read/write */
# define MAIN_CNTRL0_SR           (1 << 0)
# define MAIN_CNTRL0_DECS         (1 << 1)
# define MAIN_CNTRL0_DEHS         (1 << 2)
# define MAIN_CNTRL0_CECS         (1 << 3)
# define MAIN_CNTRL0_CEHS         (1 << 4)
# define MAIN_CNTRL0_SCALER       (1 << 7)
#define REG_VERSION_MSB           REG(0x00, 0x02)     /* read */
#define REG_SOFTRESET             REG(0x00, 0x0a)     /* write */
# define SOFTRESET_AUDIO          (1 << 0)
# define SOFTRESET_I2C_MASTER     (1 << 1)
#define REG_DDC_DISABLE           REG(0x00, 0x0b)     /* read/write */
#define REG_CCLK_ON               REG(0x00, 0x0c)     /* read/write */
#define REG_I2C_MASTER            REG(0x00, 0x0d)     /* read/write */
# define I2C_MASTER_DIS_MM        (1 << 0)
# define I2C_MASTER_DIS_FILT      (1 << 1)
# define I2C_MASTER_APP_STRT_LAT  (1 << 2)
#define REG_INT_FLAGS_0           REG(0x00, 0x0f)     /* read/write */
#define REG_INT_FLAGS_1           REG(0x00, 0x10)     /* read/write */
#define REG_INT_FLAGS_2           REG(0x00, 0x11)     /* read/write */
# define INT_FLAGS_2_EDID_BLK_RD  (1 << 1)
#define REG_ENA_VP_0              REG(0x00, 0x18)     /* read/write */
#define REG_ENA_VP_1              REG(0x00, 0x19)     /* read/write */
#define REG_ENA_VP_2              REG(0x00, 0x1a)     /* read/write */
#define REG_ENA_AP                REG(0x00, 0x1e)     /* read/write */
#define REG_VIP_CNTRL_0           REG(0x00, 0x20)     /* write */
# define VIP_CNTRL_0_MIRR_A       (1 << 7)
# define VIP_CNTRL_0_SWAP_A(x)    (((x) & 7) << 4)
# define VIP_CNTRL_0_MIRR_B       (1 << 3)
# define VIP_CNTRL_0_SWAP_B(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_1           REG(0x00, 0x21)     /* write */
# define VIP_CNTRL_1_MIRR_C       (1 << 7)
# define VIP_CNTRL_1_SWAP_C(x)    (((x) & 7) << 4)
# define VIP_CNTRL_1_MIRR_D       (1 << 3)
# define VIP_CNTRL_1_SWAP_D(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_2           REG(0x00, 0x22)     /* write */
# define VIP_CNTRL_2_MIRR_E       (1 << 7)
# define VIP_CNTRL_2_SWAP_E(x)    (((x) & 7) << 4)
# define VIP_CNTRL_2_MIRR_F       (1 << 3)
# define VIP_CNTRL_2_SWAP_F(x)    (((x) & 7) << 0)
#define REG_VIP_CNTRL_3           REG(0x00, 0x23)     /* write */
# define VIP_CNTRL_3_X_TGL        (1 << 0)
# define VIP_CNTRL_3_H_TGL        (1 << 1)
# define VIP_CNTRL_3_V_TGL        (1 << 2)
# define VIP_CNTRL_3_EMB          (1 << 3)
# define VIP_CNTRL_3_SYNC_DE      (1 << 4)
# define VIP_CNTRL_3_SYNC_HS      (1 << 5)
# define VIP_CNTRL_3_DE_INT       (1 << 6)
# define VIP_CNTRL_3_EDGE         (1 << 7)
#define REG_VIP_CNTRL_4           REG(0x00, 0x24)     /* write */
# define VIP_CNTRL_4_BLC(x)       (((x) & 3) << 0)
# define VIP_CNTRL_4_BLANKIT(x)   (((x) & 3) << 2)
# define VIP_CNTRL_4_CCIR656      (1 << 4)
# define VIP_CNTRL_4_656_ALT      (1 << 5)
# define VIP_CNTRL_4_TST_656      (1 << 6)
# define VIP_CNTRL_4_TST_PAT      (1 << 7)
#define REG_VIP_CNTRL_5           REG(0x00, 0x25)     /* write */
# define VIP_CNTRL_5_CKCASE       (1 << 0)
# define VIP_CNTRL_5_SP_CNT(x)    (((x) & 3) << 1)
#define REG_MUX_AP                REG(0x00, 0x26)
#define MUX_AP_SELECT_I2S        (0x64)
#define REG_MAT_CONTRL            REG(0x00, 0x80)     /* write */
# define MAT_CONTRL_MAT_SC(x)     (((x) & 3) << 0)
# define MAT_CONTRL_MAT_BP        (1 << 2)
#define REG_VIDFORMAT             REG(0x00, 0xa0)     /* write */
#define REG_REFPIX_MSB            REG(0x00, 0xa1)     /* write */
#define REG_REFPIX_LSB            REG(0x00, 0xa2)     /* write */
#define REG_REFLINE_MSB           REG(0x00, 0xa3)     /* write */
#define REG_REFLINE_LSB           REG(0x00, 0xa4)     /* write */
#define REG_NPIX_MSB              REG(0x00, 0xa5)     /* write */
#define REG_NPIX_LSB              REG(0x00, 0xa6)     /* write */
#define REG_NLINE_MSB             REG(0x00, 0xa7)     /* write */
#define REG_NLINE_LSB             REG(0x00, 0xa8)     /* write */
#define REG_VS_LINE_STRT_1_MSB    REG(0x00, 0xa9)     /* write */
#define REG_VS_LINE_STRT_1_LSB    REG(0x00, 0xaa)     /* write */
#define REG_VS_PIX_STRT_1_MSB     REG(0x00, 0xab)     /* write */
#define REG_VS_PIX_STRT_1_LSB     REG(0x00, 0xac)     /* write */
#define REG_VS_LINE_END_1_MSB     REG(0x00, 0xad)     /* write */
#define REG_VS_LINE_END_1_LSB     REG(0x00, 0xae)     /* write */
#define REG_VS_PIX_END_1_MSB      REG(0x00, 0xaf)     /* write */
#define REG_VS_PIX_END_1_LSB      REG(0x00, 0xb0)     /* write */
#define REG_VS_PIX_STRT_2_MSB     REG(0x00, 0xb3)     /* write */
#define REG_VS_PIX_STRT_2_LSB     REG(0x00, 0xb4)     /* write */
#define REG_VS_PIX_END_2_MSB      REG(0x00, 0xb7)     /* write */
#define REG_VS_PIX_END_2_LSB      REG(0x00, 0xb8)     /* write */
#define REG_HS_PIX_START_MSB      REG(0x00, 0xb9)     /* write */
#define REG_HS_PIX_START_LSB      REG(0x00, 0xba)     /* write */
#define REG_HS_PIX_STOP_MSB       REG(0x00, 0xbb)     /* write */
#define REG_HS_PIX_STOP_LSB       REG(0x00, 0xbc)     /* write */
#define REG_VWIN_START_1_MSB      REG(0x00, 0xbd)     /* write */
#define REG_VWIN_START_1_LSB      REG(0x00, 0xbe)     /* write */
#define REG_VWIN_END_1_MSB        REG(0x00, 0xbf)     /* write */
#define REG_VWIN_END_1_LSB        REG(0x00, 0xc0)     /* write */
#define REG_DE_START_MSB          REG(0x00, 0xc5)     /* write */
#define REG_DE_START_LSB          REG(0x00, 0xc6)     /* write */
#define REG_DE_STOP_MSB           REG(0x00, 0xc7)     /* write */
#define REG_DE_STOP_LSB           REG(0x00, 0xc8)     /* write */
#define REG_TBG_CNTRL_0           REG(0x00, 0xca)     /* write */
# define TBG_CNTRL_0_FRAME_DIS    (1 << 5)
# define TBG_CNTRL_0_SYNC_MTHD    (1 << 6)
# define TBG_CNTRL_0_SYNC_ONCE    (1 << 7)
#define REG_TBG_CNTRL_1           REG(0x00, 0xcb)     /* write */
# define TBG_CNTRL_1_VH_TGL_0     (1 << 0)
# define TBG_CNTRL_1_VH_TGL_1     (1 << 1)
# define TBG_CNTRL_1_VH_TGL_2     (1 << 2)
# define TBG_CNTRL_1_VHX_EXT_DE   (1 << 3)
# define TBG_CNTRL_1_VHX_EXT_HS   (1 << 4)
# define TBG_CNTRL_1_VHX_EXT_VS   (1 << 5)
# define TBG_CNTRL_1_DWIN_DIS     (1 << 6)
#define REG_ENABLE_SPACE          REG(0x00, 0xd6)     /* write */
#define REG_HVF_CNTRL_0           REG(0x00, 0xe4)     /* write */
# define HVF_CNTRL_0_SM           (1 << 7)
# define HVF_CNTRL_0_RWB          (1 << 6)
# define HVF_CNTRL_0_PREFIL(x)    (((x) & 3) << 2)
# define HVF_CNTRL_0_INTPOL(x)    (((x) & 3) << 0)
#define REG_HVF_CNTRL_1           REG(0x00, 0xe5)     /* write */
# define HVF_CNTRL_1_FOR          (1 << 0)
# define HVF_CNTRL_1_YUVBLK       (1 << 1)
# define HVF_CNTRL_1_VQR(x)       (((x) & 3) << 2)
# define HVF_CNTRL_1_PAD(x)       (((x) & 3) << 4)
# define HVF_CNTRL_1_SEMI_PLANAR  (1 << 6)
#define REG_RPT_CNTRL             REG(0x00, 0xf0)     /* write */
#define REG_I2S_FORMAT            REG(0x00, 0xfc)

#define REG_AIP_CLKSEL            REG(0x00, 0xfd)
# define SEL_AIP_I2S              (1 << 3)  /* I2S Clk */


/* Page 02h: PLL settings */
#define REG_PLL_SERIAL_1          REG(0x02, 0x00)     /* read/write */
# define PLL_SERIAL_1_SRL_FDN     (1 << 0)
# define PLL_SERIAL_1_SRL_IZ(x)   (((x) & 3) << 1)
# define PLL_SERIAL_1_SRL_MAN_IZ  (1 << 6)
#define REG_PLL_SERIAL_2          REG(0x02, 0x01)     /* read/write */
# define PLL_SERIAL_2_SRL_NOSC(x) (((x) & 3) << 0)
# define PLL_SERIAL_2_SRL_PR(x)   (((x) & 0xf) << 4)
#define REG_PLL_SERIAL_3          REG(0x02, 0x02)     /* read/write */
# define PLL_SERIAL_3_SRL_CCIR    (1 << 0)
# define PLL_SERIAL_3_SRL_DE      (1 << 2)
# define PLL_SERIAL_3_SRL_PXIN_SEL (1 << 4)
#define REG_SERIALIZER            REG(0x02, 0x03)     /* read/write */
#define REG_BUFFER_OUT            REG(0x02, 0x04)     /* read/write */
#define REG_PLL_SCG1              REG(0x02, 0x05)     /* read/write */
#define REG_PLL_SCG2              REG(0x02, 0x06)     /* read/write */
#define REG_PLL_SCGN1             REG(0x02, 0x07)     /* read/write */
#define REG_PLL_SCGN2             REG(0x02, 0x08)     /* read/write */
#define REG_PLL_SCGR1             REG(0x02, 0x09)     /* read/write */
#define REG_PLL_SCGR2             REG(0x02, 0x0a)     /* read/write */
#define REG_AUDIO_DIV             REG(0x02, 0x0e)     /* read/write */
#define REG_SEL_CLK               REG(0x02, 0x11)     /* read/write */
# define SEL_CLK_SEL_CLK1         (1 << 0)
# define SEL_CLK_SEL_VRF_CLK(x)   (((x) & 3) << 1)
# define SEL_CLK_ENA_SC_CLK       (1 << 3)
#define REG_ANA_GENERAL           REG(0x02, 0x12)     /* read/write */


/* Page 09h: EDID Control */
#define REG_EDID_DATA_0           REG(0x09, 0x00)     /* read */
/* next 127 successive registers are the EDID block */
#define REG_EDID_CTRL             REG(0x09, 0xfa)     /* read/write */
#define REG_DDC_ADDR              REG(0x09, 0xfb)     /* read/write */
#define REG_DDC_OFFS              REG(0x09, 0xfc)     /* read/write */
#define REG_DDC_SEGM_ADDR         REG(0x09, 0xfd)     /* read/write */
#define REG_DDC_SEGM              REG(0x09, 0xfe)     /* read/write */


/* Page 10h: information frames and packets */

#define REG_AVI_IF                REG(0x10, 0x40)   /* AVI Infoframe packet */
#define REG_AUDIO_IF              REG(0x10, 0x80)   /* AVI Infoframe packet */


/* Page 11h: audio settings and content info packets */
#define REG_AIP_CNTRL_0           REG(0x11, 0x00)     /* read/write */
# define AIP_CNTRL_0_RST_FIFO     (1 << 0)
# define AIP_CNTRL_0_SWAP         (1 << 1)
# define AIP_CNTRL_0_LAYOUT       (1 << 2)
# define AIP_CNTRL_0_ACR_MAN      (1 << 5)
# define AIP_CNTRL_0_RST_CTS      (1 << 6)
#define REG_ACR_CTS_0             REG(0x11, 0x05)
#define REG_ACR_CTS_1             REG(0x11, 0x06)
#define REG_ACR_CTS_2             REG(0x11, 0x07)
#define REG_ACR_N_0               REG(0x11, 0x08)
#define REG_ACR_N_1               REG(0x11, 0x09)
#define REG_ACR_N_2               REG(0x11, 0x0a)
#define REG_GC_AVMUTE             REG(0x11, 0x0b)
# define GC_AVMUTE_CLRMUTE        (1 << 0)
# define GC_AVMUTE_SETMUTE        (1 << 1)
#define REG_CTS_N                 REG(0x11, 0x0c)

#define REG_ENC_CNTRL             REG(0x11, 0x0d)     /* read/write */
# define ENC_CNTRL_RST_ENC        (1 << 0)
# define ENC_CNTRL_RST_SEL        (1 << 1)
# define ENC_CNTRL_CTL_CODE(x)    (((x) & 3) << 2)


#define REG_DIP_FLAGS             REG(0x11, 0x0e)
# define DIP_FLAGS_ACR            (1 << 0)
#define REG_DIP_IF_FLAGS          REG(0x11, 0x0f)     /* read/write */
#define DIP_IF_FLAGS_IF1          (1 << 1)
#define DIP_IF_FLAGS_IF2          (1 << 2)
#define DIP_IF_FLAGS_IF3          (1 << 3)
#define DIP_IF_FLAGS_IF4          (1 << 4)
#define DIP_IF_FLAGS_IF5          (1 << 5)


/* Page 12h: HDCP and OTP */
#define REG_TX3                   REG(0x12, 0x9a)     /* read/write */
#define REG_TX33                  REG(0x12, 0xb8)     /* read/write */
# define TX33_HDMI                (1 << 1)


/* Page 13h: Gamut related metadata packets */



/* CEC registers: (not paged)
 */
#define REG_CEC_FRO_IM_CLK_CTRL   0xfb                /* read/write */
#define CEC_FRO_IM_CLK_CTRL_GHOST_DIS (1 << 7)
#define CEC_FRO_IM_CLK_CTRL_ENA_OTP   (1 << 6)
#define CEC_FRO_IM_CLK_CTRL_IMCLK_SEL (1 << 1)
#define CEC_FRO_IM_CLK_CTRL_FRO_DIV   (1 << 0)
#define REG_CEC_RXSHPDLEV         0xfe                /* read */
#define CEC_RXSHPDLEV_RXSENS     (1 << 0)
#define CEC_RXSHPDLEV_HPD        (1 << 1)

#define REG_CEC_ENAMODS           0xff                /* read/write */
#define CEC_ENAMODS_DIS_FRO      (1 << 6)
#define CEC_ENAMODS_DIS_CCLK     (1 << 5)
#define CEC_ENAMODS_EN_RXSENS    (1 << 2)
#define CEC_ENAMODS_EN_HDMI      (1 << 1)
#define CEC_ENAMODS_EN_CEC       (1 << 0)


/* Device versions: */
#define TDA9989N2                 0x0101
#define TDA19989                  0x0201
#define TDA19989N2                0x0202
#define TDA19988                  0x0301

struct drm_display_mode display_mode_1280_720
	/* CEA 4 - 1280x720@60Hz */
	= { DRM_MODE("1280x720", DRM_MODE_TYPE_DRIVER, 74250, 1280, 1390,
		   1430, 1650, 0, 720, 725, 730, 750, 0,
		   DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC) };

struct drm_display_mode display_mode_1024_768
	/* 1024x768@60Hz */
	=	{ DRM_MODE("1024x768", DRM_MODE_TYPE_DRIVER, 65000, 1024, 1048,
			1184, 1344, 0, 768, 771, 777, 806, 0,
			DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) };

struct drm_display_mode display_mode_640_480
	/* CEA 1 - 640x480@60Hz */
	=	{ DRM_MODE("640x480", DRM_MODE_TYPE_DRIVER, 25175, 640, 656,
			752, 800, 0, 480, 490, 492, 525, 0,
			DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC) };

static uint8_t *
do_get_edid(struct i2c_client *client);

static void
cec_write(struct i2c_client *client, uint16_t addr, uint8_t val)
{
	uint8_t buf[] = {addr, val};
	int ret;

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to cec:0x%x\n", ret, addr);
}

static uint8_t
cec_read(struct i2c_client *client, uint8_t addr)
{
	uint8_t val;
	int ret;

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, &val, sizeof(val));
	if (ret < 0)
		goto fail;

	return val;

fail:
	dev_err(&client->dev, "Error %d reading from cec:0x%x\n", ret, addr);
	return 0;
}

static void
set_page(struct i2c_client *client, uint16_t reg)
{
	uint8_t buf[] = {
			REG_CURPAGE, REG2PAGE(reg)
	};
	int ret = i2c_master_send(client, buf, sizeof(buf));
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to REG_CURPAGE\n", ret);
}

static int
reg_read_range(struct i2c_client *client, uint16_t reg, char *buf, int cnt)
{
	uint8_t addr = REG2ADDR(reg);
	int ret;

	set_page(client, reg);

	ret = i2c_master_send(client, &addr, sizeof(addr));
	if (ret < 0)
		goto fail;

	ret = i2c_master_recv(client, buf, cnt);
	if (ret < 0)
		goto fail;

	return ret;

fail:
	dev_err(&client->dev, "Error %d reading from 0x%x\n", ret, reg);
	return ret;
}

static int
reg_write_range(struct i2c_client *client, uint16_t reg, char *buf, int cnt)
{
	int ret = 0;
	uint8_t *i2cpacket;

	i2cpacket = (uint8_t *)kmalloc(cnt + 1, GFP_KERNEL);
	if(!i2cpacket) {
		goto fail;
	}
	i2cpacket[0] = REG2ADDR(reg);
	memcpy(&i2cpacket[1], buf, cnt);

	set_page(client, reg);

	ret = i2c_master_send(client, i2cpacket, cnt+1);
	kfree(i2cpacket);
	if (ret < 0)
		goto fail;

	return ret;

fail:
	dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
	return ret;
}

static uint8_t
reg_read(struct i2c_client *client, uint16_t reg)
{
	uint8_t val = 0;
	reg_read_range(client, reg, &val, sizeof(val));
	return val;
}

static void
reg_write(struct i2c_client *client, uint16_t reg, uint8_t val)
{
	uint8_t buf[] = {REG2ADDR(reg), val};
	int ret;

	set_page(client, reg);

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
}

static void
reg_write16(struct i2c_client *client, uint16_t reg, uint16_t val)
{
	uint8_t buf[] = {REG2ADDR(reg), val >> 8, val};
	int ret;

	set_page(client, reg);

	ret = i2c_master_send(client, buf, ARRAY_SIZE(buf));
	if (ret < 0)
		dev_err(&client->dev, "Error %d writing to 0x%x\n", ret, reg);
}

static void
reg_set(struct i2c_client *client, uint16_t reg, uint8_t val)
{
	reg_write(client, reg, reg_read(client, reg) | val);
}

static void
reg_clear(struct i2c_client *client, uint16_t reg, uint8_t val)
{
	reg_write(client, reg, reg_read(client, reg) & ~val);
}

static void
tda998x_reset(struct i2c_client *client)
{
	/* reset audio and i2c master: */
	reg_set(client, REG_SOFTRESET, SOFTRESET_AUDIO | SOFTRESET_I2C_MASTER);
	msleep(50);
	reg_clear(client, REG_SOFTRESET, SOFTRESET_AUDIO | SOFTRESET_I2C_MASTER);
	msleep(50);

        /* reset transmitter: */
	reg_set(client, REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);
	reg_clear(client, REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);

        /* PLL registers common configuration */
	reg_write(client, REG_PLL_SERIAL_1, 0x00);
	reg_write(client, REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(1));
	reg_write(client, REG_PLL_SERIAL_3, 0x00);
	reg_write(client, REG_SERIALIZER,   0x00);
	reg_write(client, REG_BUFFER_OUT,   0x00);
	reg_write(client, REG_PLL_SCG1,     0x00);
	reg_write(client, REG_AUDIO_DIV,    0x03);
	reg_write(client, REG_SEL_CLK,      SEL_CLK_SEL_CLK1 | SEL_CLK_ENA_SC_CLK);
	reg_write(client, REG_PLL_SCGN1,    0xfa);
	reg_write(client, REG_PLL_SCGN2,    0x00);
	reg_write(client, REG_PLL_SCGR1,    0x5b);
	reg_write(client, REG_PLL_SCGR2,    0x00);
	reg_write(client, REG_PLL_SCG2,     0x10);
}

/* DRM encoder functions */

static void
tda998x_encoder_dpms(struct i2c_client *client, int mode)
{
	/* we only care about on or off: */
	if (mode != DRM_MODE_DPMS_ON)
		mode = DRM_MODE_DPMS_OFF;
	switch (mode) {
	case DRM_MODE_DPMS_ON:
		/* enable audio and video ports */
		reg_write(client, REG_ENA_AP, 0x03);
		reg_write(client, REG_ENA_VP_0, 0xff);
		reg_write(client, REG_ENA_VP_1, 0xff);
		reg_write(client, REG_ENA_VP_2, 0xff);

		/* set muxing after enabling ports: */
		reg_write(client, REG_VIP_CNTRL_0,
				VIP_CNTRL_0_SWAP_A(2) | VIP_CNTRL_0_SWAP_B(3));
		reg_write(client, REG_VIP_CNTRL_1,
				VIP_CNTRL_1_SWAP_C(0) | VIP_CNTRL_1_SWAP_D(1));
		reg_write(client, REG_VIP_CNTRL_2,
				VIP_CNTRL_2_SWAP_E(4) | VIP_CNTRL_2_SWAP_F(5));
		break;
	case DRM_MODE_DPMS_OFF:
		/* disable audio and video ports */
		reg_write(client, REG_ENA_AP, 0x00);
		reg_write(client, REG_ENA_VP_0, 0x00);
		reg_write(client, REG_ENA_VP_1, 0x00);
		reg_write(client, REG_ENA_VP_2, 0x00);
		break;
	}
}

static void
tda998x_audio_infoframe_enable(struct i2c_client *client)
{
	uint8_t buffer[20];
	struct hdmi_audio_infoframe audio_frame;
	size_t len;

	hdmi_audio_infoframe_init(&audio_frame);

	/* NXP audio is fixed at these values for the time being */
	audio_frame.channels = 2;
	audio_frame.coding_type = HDMI_AUDIO_CODING_TYPE_PCM;
	audio_frame.sample_size = HDMI_AUDIO_SAMPLE_SIZE_24;
	audio_frame.sample_frequency = HDMI_AUDIO_SAMPLE_FREQUENCY_48000;

	len = hdmi_audio_infoframe_pack(&audio_frame, buffer, sizeof(buffer));
	WARN(len < 0, "hdmi_avi_infoframe_pack failed\n");

	reg_write_range(client, REG_AUDIO_IF, buffer, len);

	/* enable Audio Infoframe output in DIP_IF Register */
	reg_clear(client, REG_DIP_IF_FLAGS, DIP_IF_FLAGS_IF4);
	udelay(5);
	reg_set(client, REG_DIP_IF_FLAGS, DIP_IF_FLAGS_IF4);
}

static void
tda998x_avi_infoframe_enable(struct i2c_client *client,
			struct drm_display_mode *mode)
{
	uint8_t buffer[20];
	struct hdmi_avi_infoframe avi_frame;
	size_t len;

	hdmi_avi_infoframe_init(&avi_frame);
	avi_frame.video_code = drm_match_cea_mode(mode);
	avi_frame.picture_aspect = HDMI_PICTURE_ASPECT_NONE;
	avi_frame.active_aspect = HDMI_ACTIVE_ASPECT_PICTURE;
	len = hdmi_avi_infoframe_pack(&avi_frame, buffer, sizeof(buffer));
	WARN(len < 0, "hdmi_avi_infoframe_pack failed\n");

	reg_write_range(client, REG_AVI_IF, buffer, len);

	/*
	 * enable AVI Infoframe output in DIP_IF Register, but toggle it
	 * so that the hardware acknowledges that the packet data might have
	 * changed
	 */
	reg_clear(client, REG_DIP_IF_FLAGS, DIP_IF_FLAGS_IF3);
	udelay(5);
	reg_set(client, REG_DIP_IF_FLAGS, DIP_IF_FLAGS_IF3);
}

/* loopup table for CEA values to VIDFORMAT values taken from NXP datasheet */
static char cea_to_nxp_mode[32] = {-1, 0, 1, 1, 2, 3, 4, 4, 5, 5, -1, -1,
		-1, -1, -1, -1, 6, 7, 7, 8, 9, 10, 10,
		11, 11, -1, -1, -1, -1, -1, -1, 12};

static char tda998x_cea_to_vidformat(unsigned char cea_mode)
{
	if(cea_mode > 31) {
		return -1;
	}
	return cea_to_nxp_mode[cea_mode];
}

static char tda998x_is_monitor_hdmi(struct i2c_client *client)
{
	struct edid *edid = (struct edid *)do_get_edid(client);
	char hdmi = 0;
	if(edid) {
		hdmi = drm_detect_hdmi_monitor(edid);
		kfree(edid);
	} else {
		return -1;
	}
	return hdmi;
}


static void
tda998x_encoder_mode_set(struct i2c_client *client)
{
	uint16_t hs_start, hs_end, line_start, line_end;
	uint16_t vwin_start, vwin_end, de_start, de_end;
	uint16_t ref_pix, ref_line, pix_start2;
	uint8_t reg, div, rep;
	struct drm_display_mode *mode = &display_mode_1280_720;

	hs_start   = mode->hsync_start - mode->hdisplay;
	hs_end     = mode->hsync_end - mode->hdisplay;
	line_start = 1;
	line_end   = 1 + mode->vsync_end - mode->vsync_start;
	vwin_start = mode->vtotal - mode->vsync_start;
	vwin_end   = vwin_start + mode->vdisplay;
	de_start   = mode->htotal - mode->hdisplay;
	de_end     = mode->htotal;

	pix_start2 = 0;
	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		pix_start2 = (mode->htotal / 2) + hs_start;

	/* TODO how is this value calculated?  It is 2 for all common
	 * formats in the tables in out of tree nxp driver (assuming
	 * I've properly deciphered their byzantine table system)
	 */
	ref_line = 2;

	/* this might changes for other color formats from the CRTC: */
	ref_pix = 3 + hs_start;

	div = 148500 / mode->clock;

	DBG("clock=%d, div=%u", mode->clock, div);
	DBG("hs_start=%u, hs_end=%u, line_start=%u, line_end=%u",
			hs_start, hs_end, line_start, line_end);
	DBG("vwin_start=%u, vwin_end=%u, de_start=%u, de_end=%u",
			vwin_start, vwin_end, de_start, de_end);
	DBG("ref_line=%u, ref_pix=%u, pix_start2=%u",
			ref_line, ref_pix, pix_start2);

	/* mute the audio FIFO: */
	reg_set(client, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);

	/* set HDMI HDCP mode off: */
	reg_set(client, REG_TBG_CNTRL_1, TBG_CNTRL_1_DWIN_DIS);
	reg_clear(client, REG_TX33, TX33_HDMI);

	reg_write(client, REG_ENC_CNTRL, ENC_CNTRL_CTL_CODE(0));
	/* no pre-filter or interpolator: */
	reg_write(client, REG_HVF_CNTRL_0, HVF_CNTRL_0_PREFIL(0) |
			HVF_CNTRL_0_INTPOL(0));
	reg_write(client, REG_VIP_CNTRL_5, VIP_CNTRL_5_SP_CNT(0));
	reg_write(client, REG_VIP_CNTRL_4, VIP_CNTRL_4_BLANKIT(0) |
			VIP_CNTRL_4_BLC(0));
	reg_clear(client, REG_PLL_SERIAL_3, PLL_SERIAL_3_SRL_CCIR);

	reg_clear(client, REG_PLL_SERIAL_1, PLL_SERIAL_1_SRL_MAN_IZ);
	reg_clear(client, REG_PLL_SERIAL_3, PLL_SERIAL_3_SRL_DE);
	reg_write(client, REG_SERIALIZER, 0);
/*
 *	Following the original value from NXP driver
	reg_write(client, REG_HVF_CNTRL_1, HVF_CNTRL_1_VQR(0));
*/
	reg_write(client, REG_HVF_CNTRL_1, HVF_CNTRL_1_VQR(1));

	/* TODO enable pixel repeat for pixel rates less than 25Msamp/s */
	rep = 0;
	reg_write(client, REG_RPT_CNTRL, 0);
	reg_write(client, REG_SEL_CLK, SEL_CLK_SEL_VRF_CLK(0) |
			SEL_CLK_SEL_CLK1 | SEL_CLK_ENA_SC_CLK);
	reg_write(client, REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(div) |
			PLL_SERIAL_2_SRL_PR(rep));
	reg_write(client, REG_SEL_CLK, 0x00);
	reg_write(client, REG_PLL_SERIAL_2, 0x01);


	reg_write16(client, REG_VS_PIX_STRT_2_MSB, pix_start2);
	reg_write16(client, REG_VS_PIX_END_2_MSB, pix_start2);

	/* set color matrix bypass flag: */
	reg_set(client, REG_MAT_CONTRL, MAT_CONTRL_MAT_BP);

	/* set BIAS tmds value: */
	reg_write(client, REG_ANA_GENERAL, 0x09);

	reg_clear(client, REG_TBG_CNTRL_0, TBG_CNTRL_0_SYNC_MTHD);

	reg_write(client, REG_VIP_CNTRL_3, 0);
	reg_set(client, REG_VIP_CNTRL_3, VIP_CNTRL_3_SYNC_HS);
	if (mode->flags & DRM_MODE_FLAG_NVSYNC)
		reg_set(client, REG_VIP_CNTRL_3, VIP_CNTRL_3_V_TGL);

	if (mode->flags & DRM_MODE_FLAG_NHSYNC)
		reg_set(client, REG_VIP_CNTRL_3, VIP_CNTRL_3_H_TGL);

	if(tda998x_is_monitor_hdmi(client) == 1) {
		char vidformat;
		vidformat = tda998x_cea_to_vidformat(drm_match_cea_mode(mode));
		if(vidformat == (char)-1) {
			dev_err(&client->dev, "Not sure which CEA mode to set, leaving as DVI");
			reg_write(client, REG_VIDFORMAT, 0);
			goto out;
		}
		dev_info(&client->dev, "Connected to an HDMI monitor with cea mode %d", vidformat);

		/* this is an HDMI monitor, so set things up a bit differently */
		if (mode->flags & (DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC))
			reg_set(client, REG_TBG_CNTRL_1, TBG_CNTRL_1_VH_TGL_0);
		else
			reg_write(client, REG_TBG_CNTRL_1, 0);

		reg_write(client, REG_VIDFORMAT, vidformat);
		/* get the infoframes pumping */
		tda998x_avi_infoframe_enable(client, mode);
		tda998x_audio_infoframe_enable(client);

/*HDCP  HDCP mode is turned off since a violet vertical
 * line appears at the left corner
 *		reg_set(client, REG_TX33, TX33_HDMI);
 */
		/* set up audio registers */
		reg_write(client, REG_ACR_CTS_0, 0x0);
		reg_write(client, REG_ACR_CTS_1, 0x0);
		reg_write(client, REG_ACR_CTS_2, 0x0);

		reg_write(client, REG_ACR_N_0, 0x0);
		reg_write(client, REG_ACR_N_1, 0x18);
		reg_write(client, REG_ACR_N_2, 0x0);

		reg_set(client, REG_DIP_FLAGS, DIP_FLAGS_ACR);

		reg_write(client, REG_ENC_CNTRL, 0x04);
		reg_write(client, REG_CTS_N, 0x33);
		/* Set 2 channel I2S mode */
		reg_write(client, REG_ENA_AP, 0x3);

		/* set audio divider in pll settings */
		reg_write(client, REG_AUDIO_DIV, 0x2);

		/* select the audio input port clock */
		reg_write(client, REG_AIP_CLKSEL, SEL_AIP_I2S);
		reg_write(client, REG_MUX_AP, MUX_AP_SELECT_I2S);

		/* select I2S format, and datasize */
		reg_write(client, REG_I2S_FORMAT, 0x0a);

		/* enable the audio FIFO: */
		reg_clear(client, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);

		/* mute and then unmute, to get audio going */
		reg_write(client, REG_GC_AVMUTE, GC_AVMUTE_SETMUTE);
		reg_write(client, REG_GC_AVMUTE, GC_AVMUTE_CLRMUTE);

	}

out:
	reg_write16(client, REG_NPIX_MSB, mode->hdisplay - 1);
	reg_write16(client, REG_NLINE_MSB, mode->vdisplay - 1);
	reg_write16(client, REG_VS_LINE_STRT_1_MSB, line_start);
	reg_write16(client, REG_VS_LINE_END_1_MSB, line_end);
	reg_write16(client, REG_VS_PIX_STRT_1_MSB, hs_start);
	reg_write16(client, REG_VS_PIX_END_1_MSB, hs_start);
	reg_write16(client, REG_HS_PIX_START_MSB, hs_start);
	reg_write16(client, REG_HS_PIX_STOP_MSB, hs_end);
	reg_write16(client, REG_VWIN_START_1_MSB, vwin_start);
	reg_write16(client, REG_VWIN_END_1_MSB, vwin_end);
	reg_write16(client, REG_DE_START_MSB, de_start);
	reg_write16(client, REG_DE_STOP_MSB, de_end);

	/* let incoming pixels fill the active space (if any) */
	reg_write(client, REG_ENABLE_SPACE, 0x01);

	reg_write16(client, REG_REFPIX_MSB, ref_pix);
	reg_write16(client, REG_REFLINE_MSB, ref_line);

	reg = TBG_CNTRL_1_VHX_EXT_DE |
			TBG_CNTRL_1_VHX_EXT_HS |
			TBG_CNTRL_1_VHX_EXT_VS |
			TBG_CNTRL_1_DWIN_DIS | /* HDCP off */
			TBG_CNTRL_1_VH_TGL_2;
	if (mode->flags & (DRM_MODE_FLAG_NVSYNC | DRM_MODE_FLAG_NHSYNC))
		reg |= TBG_CNTRL_1_VH_TGL_0;
	reg_set(client, REG_TBG_CNTRL_1, reg);

	/* must be last register set: */
	reg_clear(client, REG_TBG_CNTRL_0, TBG_CNTRL_0_SYNC_ONCE);
}

static int
read_edid_block(struct i2c_client *client, uint8_t *buf, int blk)
{
	uint8_t offset, segptr;
	int ret, i;

	/* enable EDID read irq: */
	reg_set(client, REG_INT_FLAGS_2, INT_FLAGS_2_EDID_BLK_RD);

	offset = (blk & 1) ? 128 : 0;
	segptr = blk / 2;

	reg_write(client, REG_DDC_ADDR, 0xa0);
	reg_write(client, REG_DDC_OFFS, offset);
	reg_write(client, REG_DDC_SEGM_ADDR, 0x60);
	reg_write(client, REG_DDC_SEGM, segptr);

	/* enable reading EDID: */
	reg_write(client, REG_EDID_CTRL, 0x1);

	/* flag must be cleared by sw: */
	reg_write(client, REG_EDID_CTRL, 0x0);

	/* wait for block read to complete: */
	for (i = 100; i > 0; i--) {
		uint8_t val = reg_read(client, REG_INT_FLAGS_2);
		if (val & INT_FLAGS_2_EDID_BLK_RD)
			break;
		msleep(1);
	}

	if (i == 0)
		return -ETIMEDOUT;

	ret = reg_read_range(client, REG_EDID_DATA_0, buf, EDID_LENGTH);
	if (ret != EDID_LENGTH) {
		dev_err(&client->dev, "failed to read edid block %d: %d",
				blk, ret);
		return ret;
	}

	reg_clear(client, REG_INT_FLAGS_2, INT_FLAGS_2_EDID_BLK_RD);

	return 0;
}
static uint8_t *
do_get_edid(struct i2c_client *client)
{
	int j = 0, valid_extensions = 0;
	uint8_t *block, *new;
	bool print_bad_edid = drm_debug & DRM_UT_KMS;

	if ((block = kmalloc(EDID_LENGTH, GFP_KERNEL)) == NULL)
		return NULL;
	/* base block fetch */
	if (read_edid_block(client, block, 0))
		goto fail;
	/* if there's no extensions, we're done */
	if (block[0x7e] == 0) {
		if (!drm_edid_is_valid(block)) {
			goto fail;
		}
		return block;
	}

	new = krealloc(block, (block[0x7e] + 1) * EDID_LENGTH, GFP_KERNEL);
	if (!new)
		goto fail;
	block = new;

	for (j = 1; j <= block[0x7e]; j++) {
		uint8_t *ext_block = block + (valid_extensions + 1) * EDID_LENGTH;
		if (read_edid_block(client, ext_block, j))
			goto fail;
		valid_extensions++;
	}

	if (valid_extensions != block[0x7e]) {
		block[EDID_LENGTH-1] += block[0x7e] - valid_extensions;
		block[0x7e] = valid_extensions;
		new = krealloc(block, (valid_extensions + 1) * EDID_LENGTH, GFP_KERNEL);
		if (!new)
			goto fail;
		block = new;
	}

	if (!drm_edid_is_valid(block)) {
		goto fail;
	}

	return block;

fail:
	dev_warn(&client->dev, "failed to read EDID\n");
	kfree(block);
	return NULL;
}

/* I2C driver functions */

static int
tda998x_remove(struct i2c_client *client)
{
	return 0;
}

static int
tda998x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct tda998x_priv *priv;
	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->current_page = 0;
	priv->cec = i2c_new_dummy(client->adapter, 0x34);
	priv->dpms = DRM_MODE_DPMS_OFF;

	/* wake up the device: */
	cec_write(priv->cec, REG_CEC_ENAMODS,
			CEC_ENAMODS_EN_RXSENS | CEC_ENAMODS_EN_HDMI);

	tda998x_reset(client);

	/* read version: */
	priv->rev = reg_read(client, REG_VERSION_LSB) |
			reg_read(client, REG_VERSION_MSB) << 8;

	/* mask off feature bits: */
	priv->rev &= ~0x30; /* not-hdcp and not-scalar bit */

	switch (priv->rev) {
	case TDA9989N2:  dev_info(&client->dev, "found TDA9989 n2\n");  break;
	case TDA19989:   dev_info(&client->dev, "found TDA19989\n");    break;
	case TDA19989N2: dev_info(&client->dev, "found TDA19989 n2\n"); break;
	case TDA19988:   dev_info(&client->dev, "found TDA19988\n");    break;
	default:
		dev_info(&client->dev, "found unsupported device: %04x", priv->rev);
		goto fail;
	}

	/* after reset, enable DDC: */
	reg_write(client, REG_DDC_DISABLE, 0x00);

	/* set clock on DDC channel: */
	reg_write(client, REG_TX3, 39);

	/* if necessary, disable multi-master: */
	if (priv->rev == TDA19989)
		reg_set(client, REG_I2C_MASTER, I2C_MASTER_DIS_MM);

	cec_write(priv->cec, REG_CEC_FRO_IM_CLK_CTRL,
			CEC_FRO_IM_CLK_CTRL_GHOST_DIS | CEC_FRO_IM_CLK_CTRL_IMCLK_SEL);
	tda998x_encoder_mode_set(client);
	tda998x_encoder_dpms(client,DRM_MODE_DPMS_ON);
	return 0;

fail:
	/* if encoder_init fails, the encoder slave is never registered,
	 * so cleanup here:
	 */
	if (priv->cec)
		i2c_unregister_device(priv->cec);
	kfree(priv);

	return -ENXIO;
}

/*
 *  I2C client driver (backend)
 *  -----------------
 */
static const struct i2c_device_id tda998x_ids[] = {
   { "tda998x", 0 },
   { },
};

MODULE_DEVICE_TABLE(i2c, tda998x_ids);

static struct i2c_driver this_i2c_driver = {
   .driver = {
      .owner = THIS_MODULE,
      .name = "tda998x",
   },
   .probe = tda998x_probe,
   .remove = tda998x_remove,
   .id_table = tda998x_ids,
};

/* Module initialization */

static int __init
tda998x_init(void)
{
	DBG("tda998x_init\n");
	return i2c_add_driver(&this_i2c_driver);
}

static void __exit
tda998x_exit(void)
{
	i2c_del_driver(&this_i2c_driver);
	DBG("tda998x_exit\n");
}

MODULE_AUTHOR("Rob Clark <robdclark@gmail.com");
MODULE_DESCRIPTION("NXP Semiconductors TDA998X HDMI Encoder");
MODULE_LICENSE("GPL");

module_init(tda998x_init);
module_exit(tda998x_exit);
