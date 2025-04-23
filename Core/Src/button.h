#ifndef FL003_BUTTON_H
#define FL003_BUTTON_H

#include "gui.h"

#ifdef __cplusplus
namespace button {
    enum {Left = 0, Right};
    
    const static int NumberOfButtons = 2;
    enum DetectionMode_t {
        Default = 0, // No double click detection
        DetectDoubleClick = 1 << 0,
        RespondContinuousPress = 1 << 1, // Detect long press as several short clicks
    };
    extern volatile DetectionMode_t DetectionMode[NumberOfButtons];
    
    inline DetectionMode_t operator|(DetectionMode_t a, DetectionMode_t b)
    {
        return static_cast<DetectionMode_t>(static_cast<int>(a) | static_cast<int>(b));
    }

    void Routine();
    
    bool IsPressed(int id);
}
#endif
#ifdef __cplusplus
extern "C" {
#endif

void ButtonRoutine();

#ifdef __cplusplus
}
#endif


#endif //FL003_BUTTON_H
