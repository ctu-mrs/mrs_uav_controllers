#include <nodelet/nodelet.h>

class PidController : public nodelet::Nodelet {
public:
  virtual void onInit();
};
