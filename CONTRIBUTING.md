# For developers

This repository is a ROS wrapper only and shall therefore only include ROS interfaces (service calls, test nodes, visualizations, etc.) to objects / functions defined in the [V4R library](https://github.com/strands-project/v4r) (e.g. by deriving from classes defined in the V4R library). Please do not commit any essential code into this repository - this should go directly into V4R library (so that people can use it also without ROS). This also means that external dependencies should already be resolved by the V4R library.

As a convention, please create seperate folders for service/message definitions (e.g. do not put your `*.cpp/*.hpp` files together with `*.msg/*.srv`).
In your package.xml file, please fill in the maintainer (and optional author) field with your name and e-mail address.

# License
The V4R libary and v4r\_ros\_wrappers are published under the MIT license. 

Please add this to the top of each of your files. 

```cpp
/******************************************************************************
 * Copyright (c) 2015 Author's name
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 ******************************************************************************/
```
