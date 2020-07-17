
//
// Created by Pablo Bustos on 8/5/20
//

#ifndef DSR_EXCEPTIONS
#define DSR_EXCEPTIONS

#include <exception>

/////////////////////////////////////////////////////////////////
/// DSR Exceptions
/////////////////////////////////////////////////////////////////
namespace DSR
{
    class DSRException : public std::exception
    {
        public:
            explicit DSRException(const std::string &message): msg_(message){};
            explicit DSRException(const char *message): msg_(message){};
            virtual ~DSRException() throw (){}
            virtual const char* what() const throw (){
                return msg_.c_str();
            }
        protected:
            std::string msg_;
    };
}
#endif