/*
 * MIT License
 *
 * Copyright (c) 2022 Christos Zosimidis
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *
 * debug_ct.h
 *
 *  Created on: 24.10.2022
 *      Author: Christos Zosimidis
 */

#ifndef CRAZET_DEBUG_CT_H_
#define CRAZET_DEBUG_CT_H_

#define CRAZET_DEBUG 1 //TODO: Remove this line

#ifdef CRAZET_DEBUG

void crazetDebugMessage(const char* msg);
#define CRAZET_MESSAGE(...) \
		{crazetDebugMessage(__VA_ARGS__);}

void crazetDebugWarn(const char* warn);
#define CRAZET_WARN(...) \
		{crazetDebugWarn(__VA_ARGS__);}

void crazetDebugError(const char* err);
#define CRAZET_ERROR(...) \
		{crazetDebugError(__VA_ARGS__);}

#else
#define CRAZET_MESSAGE(X)
#define CRAZET_WARNING(X)
#define CRAZET_ERROR(X)
#endif /* CRAZET_DEBUG */

#endif /* CRAZET_DEBUG_CT_H_ */
