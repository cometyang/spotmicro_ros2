


typedef struct _servo_config {
    int center;
    int range;
    int direction;
    int mode_pos;
} servo_config;

typedef struct _drive_mode {
    int mode;
    float rpm;
    
    float radius;

}