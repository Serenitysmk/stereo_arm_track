#ifndef STEREO_ARM_TRACK_CONTROLLER_H_
#define STEREO_ARM_TRACK_CONTROLLER_H_

struct ControllerOptions {
  
  bool Check() const;
};

class Controller {
 public:
  explicit Controller(const ControllerOptions* options);

 private:
  const ControllerOptions* options_;
};

#endif  // STEREO_ARM_TRACK_CONTROLLER_H_