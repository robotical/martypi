#include "data_t.h"

//----------------------------------------------------------------------------
bool data_t::load( const string& filename )
  {
  string s;
  ifstream f( filename.c_str() );
  while (getline( f, s ))
    {
    deque <float> record;
    istringstream iss( s );
    while (getline( iss, s, ',' ))
      {
      float fieldvalue = 0.0f;
      istringstream( s ) >> fieldvalue;
      record.push_back( fieldvalue );
      }
    this->push_back( record );
    }
  return f.good();
  }

//----------------------------------------------------------------------------
bool data_t::save( const string& filename )
  {
  ofstream f( filename.c_str() );
  if (!f) return false;

  return save( f );
  }

//----------------------------------------------------------------------------
bool data_t::save( ostream& outs )
  {
  for (data_t::record_iterator ri = this->begin(); ri != this->end(); ri++)
    {
    for (data_t::field_iterator fi = ri->begin(); fi != ri->end(); fi++)
      outs << ((fi == ri->begin()) ? "" : ", ") << *fi;
    outs << endl;
    }
  return outs.good();
  }

