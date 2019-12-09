#ifndef __CEI_HW_ID_H
#define __CEI_HW_ID_H

#define CEI_HWID_TYPE_STRLEN          20
/* Implement CCI HW ID/PROJECT ID */

enum CEI_PROJ_ID_TYPE {
	BT52                    = 0,
	BT53                    = 1,
	B28A                    = 2,
	BT54			= 3,
	PROJ_ID_INVALID         = 4,
	PROJ_ID_SIZE            = 5,
};

enum CEI_HW_ID_TYPE {
	EVT                      = 0,
	DVT1                     = 1,
	DVT2                     = 2,
	PVT                      = 3,
	MP                       = 4,
	HW_ID_INVALID            = 5,
	HW_ID_SIZE               = 6,
};

struct cei_hwid_data_type {
	long cei_proj_id;
	long cei_hw_id;
	long cei_qfuse;
};

extern long get_cei_proj_id(void);
extern long get_cei_hw_id(void);
extern void cei_hwid_info_read(void);

#endif /* __CCI_HW_ID_H */
