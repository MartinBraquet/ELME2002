#ifndef _GETCH_H_
#define _GETCH_H

void initTermios(int echo);
void resetTermios(void);
char getch_(int echo);
char getch(void);

#endif
