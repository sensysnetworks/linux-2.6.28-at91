#ifndef DATA_FUSION_SOM9260_H_
#define DATA_FUSION_SOM9260_H_

#ifdef CONFIG_DATA_FUSION
void data_fusion_board_init(void);
#else
#define data_fusion_board_init()
#endif

#endif /* DATA_FUSION_SOM9260_H_ */
