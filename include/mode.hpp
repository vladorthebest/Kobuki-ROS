#ifndef MODE_HPP
#define MODE_HPP

class Mode {
public:
    virtual ~Mode() = default;
    
    // Pure virtual methods that derived classes must implement
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void update() = 0;
    
protected:
    Mode() = default;
};

#endif // MODE_HPP 