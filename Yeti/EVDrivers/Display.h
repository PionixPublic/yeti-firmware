/*
 * Display.h
 *
 *  Created on: Jul 5, 2021
 *      Author: cornelius
 */

#ifndef EVDRIVERS_DISPLAY_H_
#define EVDRIVERS_DISPLAY_H_

class Display {
public:
    Display(){};

    ~Display(){};

    enum Alignment { RIGHT, LEFT, CENTER };

    virtual void clearDisplay(){};

    virtual void printText(const char *text, unsigned int column,
                           unsigned int row, Alignment align = LEFT){};

    virtual void updateDisplay(){};
};

#endif /* EVDRIVERS_DISPLAY_H_ */
