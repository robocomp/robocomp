
//
// Created by Pablo Bustos on 8/5/20
//

#ifndef DSR_EXCEPTIONS
#define DSR_EXCEPTIONS

#include <exception>
#include <utility>

/////////////////////////////////////////////////////////////////
/// DSR Exceptions
/////////////////////////////////////////////////////////////////
namespace DSR
{
    class DSRException : public std::exception
    {
        public:
            explicit DSRException(std::string message): msg_(std::move(message)){};
            explicit DSRException(const char *message): msg_(message){};
            ~DSRException() noexcept override= default;
            [[nodiscard]] const char* what() const noexcept override{
                return msg_.c_str();
            }
        protected:
            std::string msg_;
    };
}
#endif