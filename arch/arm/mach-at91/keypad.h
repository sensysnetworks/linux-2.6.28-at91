#ifndef _AT91_KEYPAD_H_
#define _AT91_KEYPAD_H_

#ifdef CONFIG_SOM_150ES
int som150_keypad_init(struct keypad_data_s *k);
#define keypad_init(k) som150_keypad_init(k)
#elif defined(CONFIG_RDAC_CARRIER)
int rdac_keypad_init(struct keypad_data_s *k);
#define keypad_init(k) rdac_keypad_init(k)
#endif

#endif /* _AT91_KEYPAD_H_ */
