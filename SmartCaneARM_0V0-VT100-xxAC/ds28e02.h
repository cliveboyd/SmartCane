/*******************************************************************************
 *	@file   ds28e02.h
 *	@device DS28e02 1-Wire SHA EEPROM
 *	@brief  Header file for DS28E02 Device Driver.
 *	@author Clive Boyd (clive.boyd@hotmail.com)
********************************************************************************

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

 */
 
#ifndef DS28E02_H
#define DS28E02_H

extern unsigned char ds28e02_id[8];
extern int ds28e02_initAndRead(void);


int ds28e02_Read_ScratchPad (void);

int ds28e02_Write_ScratchPad (void);
	
#endif /* DS28E02_H */
