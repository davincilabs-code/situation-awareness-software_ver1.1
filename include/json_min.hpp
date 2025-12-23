#pragma once
#include <string>
#include <sstream>
#include <iomanip>

namespace sa {

inline std::string esc(const std::string& s){
    std::ostringstream o;
    for(char c: s){
        switch(c){
            case '\\': o << "\\\\"; break;
            case '"':  o << "\\\""; break;
            case '\n': o << "\\n"; break;
            case '\r': o << "\\r"; break;
            case '\t': o << "\\t"; break;
            default:   o << c; break;
        }
    }
    return o.str();
}

inline std::string jnum(double v, int prec=3){
    std::ostringstream o;
    o << std::fixed << std::setprecision(prec) << v;
    return o.str();
}

} // namespace sa
