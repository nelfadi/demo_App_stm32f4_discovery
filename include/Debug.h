/*
 * Debug.h
 *
 *  Created on: 7 mars 2016
 *      Author: EL FADI
 */

#ifndef INCLUDE_DEBUG_H_
#define INCLUDE_DEBUG_H_


#ifdef DEBUG
#define DBG_PRINT(...)	do{ fprintf( stdout, __VA_ARGS__ ); } while(0 )
#else
#define DBG_PRINT(fmt, ...) do {} while (0)
#endif




#endif /* INCLUDE_DEBUG_H_ */
