/*
 * Std_Types.h
 *
 *  Created on: 27.09.2016
 *      Author: SamirAlexis
 */

#ifndef STD_TYPES_H_
#define STD_TYPES_H_

#ifndef STATUSTYPEDEFINED
#define STATUSTYPEDEFINED
typedef unsigned char StatusType; /* OSEK compliance */
#endif


#ifndef STD_ON
  #define STD_ON 1u
#endif
#ifndef STD_OFF
  #define STD_OFF 0u
#endif

#ifndef E_OK
  #define E_OK 0
#endif

#ifndef E_NOT_OK
  #define E_NOT_OK 1
#endif



#endif /* STD_TYPES_H_ */
