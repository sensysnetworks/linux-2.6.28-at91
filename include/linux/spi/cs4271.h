/*
 * Board-specific data used to set up CS4271 audio codec driver.
 */

#ifndef __LINUX_SPI_CS4271_H
#define __LINUX_SPI_CS4271_H

struct cs4271_board_info {
	int		ssc_id;
	struct clk	*dac_clk;
	char		shortname[32];
};

#endif /* __LINUX_SPI_CS4271_H */
