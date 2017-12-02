

void appble_init(void(*disp_fun)(uint32_t));
void appble_start(bool erase_bonds);
void appble_adv_triggered(void);
void appble_disconn_triggered(void);
void appble_wl_off_triggered(void);
void appble_sys_evt_dispatch(uint32_t sys_evt);