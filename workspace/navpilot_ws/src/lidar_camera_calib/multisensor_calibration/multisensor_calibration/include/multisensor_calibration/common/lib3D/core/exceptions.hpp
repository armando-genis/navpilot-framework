#ifndef LIB3D_EXCEPTIONS_HPP
#define LIB3D_EXCEPTIONS_HPP

// std
#include <exception>
#include <string>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
  #define FN_NAME __FUNCTION__
#elif defined(__linux__) || defined(__unix__)
  #define FN_NAME __PRETTY_FUNCTION__
#endif

namespace lib3d {

//==================================================================================================
/**
 * @brief Exception for providing an invalid arguments to a function
 */
class InvalidArgumentException : public std::exception
{

public:

    /**
     @brief Constructor
     @param[in] iCallerFn Name of caller function.
     @param[in] iMsg Additional message to print.
     */
    explicit InvalidArgumentException(std::string iCallerFn = "",
                                      std::string iMsg = "") : std::exception(),
        mMessage(new std::string("lib3d::InvalidArgumentException"))
    {
        if(!iMsg.empty())
        {
          mMessage->append(": " + iMsg);
        }

        if(!iCallerFn.empty())
        {
          mMessage->append(" in function " + iCallerFn);
        }

        mMessage->append(".\n");
    }

    ~InvalidArgumentException() {
      if(mMessage)
        delete mMessage;
    }

    virtual const char* what() const noexcept
    {
      return mMessage->c_str();
    }

    std::string* mMessage;
};

} // namepsace lib3d

#endif // LIB3D_EXCEPTIONS_HPP
