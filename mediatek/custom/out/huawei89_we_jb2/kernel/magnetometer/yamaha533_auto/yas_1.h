/*=============================================================================*/
/* CONFIDENTIAL																   */
/* Copyright(c) 2010-2012 Yamaha Corporation								   */
/*=============================================================================*/

/*=============================================================================*/
/* File 	yas.h															   */
/* Date 	2012/03/07														   */
/* Revision	1.3.0															   */
/*=============================================================================*/

#ifndef __YAS_H__
#define __YAS_H__

#include "yas_pcb_test.h"
#include <linux/types.h>
#include <linux/list.h>

/* typedef */
struct yas_matrix {
	int32_t matrix[9];
};

struct yas_utility {
	int (*get_rotation_matrix)(struct yas_vector *acc, struct yas_vector *mag,
			struct yas_matrix *matrix);
	int (*get_euler)(struct yas_matrix *matrix, struct yas_vector *euler);
};

/* prototype functions */
#ifdef __cplusplus
extern "C" {
#endif

int yas_utility_init(struct yas_utility *f);

#ifdef __cplusplus
}
#endif

#endif /* __YAS_H__ */
