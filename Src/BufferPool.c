#include "BufferPool.h"

/**
  * @brief  Initializes the linked list structure, which are used as a buffer for motor`s control data
  * @param  start_pos 	: Pointer to first node in linked list. The function allocated the memory for
  *                				first node and recursively allocated memory for  all other next nodes.
  * @param  start_index : The number of nodes in linked list is defined by BUFFER_POOL_SIZE define. It is possible
	*												to create different number of nodes without changing the macros. Just set this argument to 
													difference between BUFFER_POOL_SIZE define and the number of nodes, that you want to generate.
													For example if BUFFER_POOL_SIZE is 10 and you want to generate linled list with 3 nodes, set
													the parameter to 10 - 3 = 8. You get the linked list with 10 notes.
  * @retval The last node in the linked list. You may set the next field in this structure to pointer to the first node.
  */
CCBuffer *cc_buffer_init(CCBuffer **start_pos, uint8_t start_index)
{
	*start_pos = malloc(sizeof(CCBuffer));
	(*start_pos)->status = FREE;
	
	if(start_index != BUFFER_POOL_SIZE)
	{
		return cc_buffer_init( &((*start_pos)->next), start_index+1);
	}
	return *start_pos;
}

/**
  * @brief  Free the allocated memory for linked list.
  * @param  start_pos 	: Pointer to first node of the linked list. It is not nesessary to pass only the 
	*												first node to this function. If you pass the node in the middle or at the end of linked list
	*												only all successive nodes will be disposed
  */
void cc_buffer_dispose(CCBuffer **start_pos)
{
	if( (*start_pos)->next != NULL )
	{
		cc_buffer_dispose( &((*start_pos)->next) );
	}
	free(*start_pos);
}
