#ifndef UCAR_SOM9X_H_
#define UCAR_SOM9X_H_

#ifdef CONFIG_UCAR_GPIO
void ucar_board_init(void);
#else
#define ucar_board_init()
#endif

#endif /* UCAR_SOM9X_H_ */
