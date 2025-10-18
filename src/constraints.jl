"""
Author: Bennet Outland
Organization: CU Boulder
Information Control: None - University Product
License: MIT
"""

# Includes

"""
The default struct for defining a constraint
"""
struct Constraint
    C::Function # function defining the constraint
    type::Symbol # type of constraint
    # Π_C::Function # Euclidean projection
end
