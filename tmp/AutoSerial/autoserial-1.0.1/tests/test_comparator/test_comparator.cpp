#include <autoserial/autoserial.h>

class SimpleObject : public autoserial::ISerializable
{
    AS_CLASSDEF(SimpleObject)
    AS_MEMBERS
        AS_ITEM(int, intItem)                  // Declare a public "int intItem;"
        AS_ITEM(double, doubleItem)            // Declare a public "double doubleItem;"
        AS_DYNARRAY(int, buf, intItem)         // Declare a public array of size "intItem"
    AS_CLASSEND;
};

int main()
{
    SimpleObject s1;                               // Instantiate and initialize a SimpleObject
    s1.doubleItem = 1.4;
    s1.intItem = 2;
    s1.buf = (int*)malloc(s1.intItem * sizeof(int));
    s1.buf[0] = 10;
    s1.buf[1] = 20;

    autoserial::OpaqueObject o;                    // Clone the simple object by serializing it
    o.set(&s1);                                    // within an OpaqueObject
    SimpleObject *s2 = (SimpleObject*)o.get();

    std::cout << "-- Comparing s1 and s2:" << std::endl;
    autoserial::Comparator c;                      // Declare a comparator
    c.printMembersEquality(&s1,s2);                // Both objects are equal

    s2->buf[1] = 30;                               // Modify buffer of s2
    s1.doubleItem += 0.001;                        // Add small difference to double
    std::cout << "-- Comparing modified s1 and s2:" << std::endl;
    c.printMembersEquality(&s1,s2);

    c.setEpsilonDouble(0.002);                     // Set higher tolerance for double comparisons
    std::cout << "-- with a larger epsilon" << std::endl;
    c.printMembersEquality(&s1,s2);

    delete s2;
    return 0;
}
