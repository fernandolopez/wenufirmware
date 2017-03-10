#ifdef NETSTACK_CONF_RDC
#undef NETSTACK_CONF_RDC
#endif
#define NETSTACK_CONF_RDC nullrdc_driver
// Si esta comentado usa CSMA
/*
#ifdef NETSTACK_CONF_MAC
#undef  NETSTACK_CONF_MAC
#endif
#define NETSTACK_CONF_MAC   nullmac_driver
*/
