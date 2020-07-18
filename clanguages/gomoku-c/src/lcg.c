//
// Created by qixiaofeng on 2020/7/18.
//

#include <time.h>

#include "lcg.h"

//! X_{n+1} = (aX_n + c) mod m
int
lcg_get()
{
    /*!
     * Only used in the context of gomoku board.
     */
    static int const modulus = 5477; //! m

    static int const multiplier = 773; //! a
    static int const increment = 7573; //! c
    static int theX = -1;
    if (theX < 0) {
        //! Initialize
        theX = time(NULL);
    }
    theX = (multiplier * theX + increment) % modulus;
    return theX;
}
