"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT
"""

# Includes

abstract type AbstractConstraint end

"""
The default struct for defining a constraint
"""
struct Constraint
    C::Function # function defining the constraint
    type::Symbol # type of constraint
end

"""
The default struct for defining a box constraint
"""
struct BoxConstraint
    upper::Vector
    lower::Vector 
    type::Symbol
end

