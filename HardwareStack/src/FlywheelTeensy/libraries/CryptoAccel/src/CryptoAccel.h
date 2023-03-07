#ifndef CryptoAccel_h_
#define CryptoAccel_h_

#ifdef __cplusplus
extern "C"{
#endif
void mmcau_aes_set_key (const unsigned char *key, const int key_size, unsigned char *key_sch);
void mmcau_aes_encrypt (const unsigned char *in, const unsigned char *key_sch, const int nr, unsigned char *out);
void mmcau_aes_decrypt (const unsigned char *in, const unsigned char *key_sch, const int nr, unsigned char *out);
int  mmcau_des_chk_parity (const unsigned char *key);
void mmcau_des_encrypt (const unsigned char *in, const unsigned char *key, unsigned char *out);
void mmcau_des_decrypt (const unsigned char *in, const unsigned char *key, unsigned char *out);
void mmcau_md5_initialize_output (const unsigned char *md5_state);
void mmcau_md5_hash_n (const unsigned char *msg_data, const int num_blks, unsigned char *md5_state);
void mmcau_md5_update (const unsigned char *msg_data, const int num_blks, unsigned char *md5_state);
void mmcau_md5_hash (const unsigned char *msg_data, unsigned char *md5_state);
void mmcau_sha1_initialize_output (const unsigned int *sha1_state);
void mmcau_sha1_hash_n (const unsigned char *msg_data, const int num_blks, unsigned int *sha1_state);
void mmcau_sha1_update (const unsigned char *msg_data, const int num_blks, unsigned int *sha1_state);
void mmcau_sha1_hash (const unsigned char *msg_data, unsigned int *sha1_state);
int  mmcau_sha256_initialize_output (const unsigned int *output);
void mmcau_sha256_hash_n (const unsigned char *input, const int num_blks, unsigned int *output);
void mmcau_sha256_update (const unsigned char *input, const int num_blks, unsigned int *output);
void mmcau_sha256_hash (const unsigned char *input, unsigned int *output);
#ifdef __cplusplus
} // extern "C"
#endif



#endif
