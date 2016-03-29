#include <nodelet/nodelet.h>

namespace bebop_img_nodelet
{

    class img_sub : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
    };

}
