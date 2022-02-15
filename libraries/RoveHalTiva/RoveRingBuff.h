/////////////////////////////////////////////////////////////////////////////////////////////////////
// MRDT 2019
/////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef ROVE_RING_BUFF_H
#define ROVE_RING_BUFF_H

/////////////////////////////////////////////////////////////////////////////////
// Todo support for NO_OVERWRITE bool arg flag??
// Todo => void pushToFront( TYPE value );
// Todo => TYPE popToBack( );
// Todo => TYPE peekFromBack(  uint32_t index );
// Todo => bool isFull();
// if( this->isFull() ) { this->tail = (this->tail + 1) % (SIZE-1);} this->cnt++;

#include <stdint.h>
#include <stddef.h>

///////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > class RoveRingBuff
{
  public:
    volatile TYPE     buff[ SIZE ];
    volatile uint32_t head =     0;
    volatile uint32_t tail =     0;
    volatile size_t   cnt  =     0;

//  TYPE   popFromBack();
    TYPE   popFromFront();
    TYPE   peekFromFront( uint32_t index=0 );
    TYPE   peekFromBack(  uint32_t index=0 );
    void   pushToBack(        TYPE value   );
//  void   pushToBack();
    bool   isAvailable();
    bool   isEmpty();
    bool   isFull();
    TYPE   average();
    TYPE   count();
    TYPE   size();
    TYPE   sum();
};

//////////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > bool RoveRingBuff< TYPE, SIZE >::isAvailable()
{ return !this->isEmpty(); }

//////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > bool RoveRingBuff< TYPE, SIZE >::isFull()
{ return ( this->cnt == SIZE ); }

//////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > bool RoveRingBuff< TYPE, SIZE >::isEmpty()
{ return ( this->cnt == 0 ); }

///////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > TYPE RoveRingBuff< TYPE, SIZE >::count()
{ return this->cnt; }

/////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > TYPE RoveRingBuff< TYPE, SIZE >::size()
{ return sizeof( this->buff ) / sizeof( this->buff[0] ); }

///////////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > TYPE RoveRingBuff< TYPE, SIZE >::popFromFront()
{ 
  if(  (              this->isEmpty() )
    || ( this->tail > this->size()    ) )
  { return ~TYPE(0); }

  else 
  { TYPE  pop_byte =  this->buff[ this->tail ];
    this->tail =                ( this->tail + 1 ) % this->size();
    this->cnt--;
    return pop_byte; }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template<class TYPE, size_t SIZE> TYPE RoveRingBuff< TYPE, SIZE >::peekFromFront( uint32_t index )
{
  if(  (              this->isEmpty() )
    || ( this->tail > this->size()    ) 
    || ( index      > this->count()   ) )
  { return ~TYPE(0); } 

  else 
  { return this->buff[ ( this->tail + index )  % this->size() ]; }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > TYPE RoveRingBuff< TYPE, SIZE >::peekFromBack( uint32_t index )
{ 
  if(  (              this->isEmpty() )
    || ( this->tail > this->size()    ) 
    || ( index      > this->count()   ) )
  { return ~TYPE(0); }

  else 
  { return this->buff[ this->head - 1 - ( index % this->size() ) ]; }
}

/////////////////////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > void RoveRingBuff< TYPE, SIZE >::pushToBack( TYPE value )
{
  this->buff[   this->head]     = value;
  this->head = (this->head + 1) % this->size();
  
  if ( this->cnt < this->size() )
  {    this->cnt++; }

  else 
  { this->tail = (this->tail + 1) % this->size(); }
}

//////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > TYPE RoveRingBuff< TYPE, SIZE >::sum()
{
  if( this->isEmpty() )
  { return ~TYPE(0); }

  else 
  {      TYPE sum  = 0;
    for( int    i  = 0; i < this->cnt;  i++ )
    {         sum +=        this->peekFromFront( i ); }
    return    sum; }
}

//////////////////////////////////////////////////////////////////////////////
template< class TYPE, size_t SIZE > TYPE RoveRingBuff< TYPE, SIZE >::average()
{
  if( this->isEmpty() )
  { return ~TYPE(0); }

  else 
  { return this->sum() / this->count(); }
}

#endif // ROVE_RING_BUFF_H