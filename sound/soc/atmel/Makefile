# AT91 Platform Support
snd-soc-atmel-pcm-objs := atmel-pcm.o
snd-soc-atmel_ssc_dai-objs := atmel_ssc_dai.o
snd-at73c213-objs		:= ../../spi/at73c213.o

obj-$(CONFIG_SND_ATMEL_SOC) += snd-soc-atmel-pcm.o
obj-$(CONFIG_SND_ATMEL_SOC_SSC) += snd-soc-atmel_ssc_dai.o
obj-$(CONFIG_SND_AT73C213)	+= snd-at73c213.o

# AT91 Machine Support
snd-soc-sam9g20-wm8731-objs := sam9g20_wm8731.o
snd-soc-som9g20-cs4271-objs := som9g20_cs4271.o


# AT32 Machine Support
snd-soc-playpaq-objs := playpaq_wm8510.o

obj-$(CONFIG_SND_AT91_SOC_SAM9G20_WM8731) += snd-soc-sam9g20-wm8731.o
obj-$(CONFIG_SND_AT91_SOC_SOM9G20_CS4271) += snd-soc-som9g20-cs4271.o
obj-$(CONFIG_SND_AT32_SOC_PLAYPAQ) += snd-soc-playpaq.o
