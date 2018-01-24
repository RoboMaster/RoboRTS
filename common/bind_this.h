#ifndef RRTS_BIND_THIS_H
#define RRTS_BIND_THIS_H
namespace std
{
template<int>  // begin with 0 here!
struct placeholder_template
{};

template<int N>
struct is_placeholder<placeholder_template<N> >
    : integral_constant<int, N+1>  // the one is important
{};
}  // end of namespace std


template<int...>
struct int_sequence
{};

template<int N, int... Is>
struct make_int_sequence
    : make_int_sequence<N-1, N-1, Is...>
{};

template<int... Is>
struct make_int_sequence<0, Is...>
    : int_sequence<Is...>
{};

template<class R, class U, class... Args, int... Is>
auto bind_this_sub(R (U::*p)(Args...), U * pp, int_sequence<Is...>)
-> decltype(std::bind(p, pp, std::placeholder_template<Is>{}...))
{
  return std::bind(p, pp, std::placeholder_template<Is>{}...);
}

// binds a member function only for the this pointer using std::bind
template<class R, class U, class... Args>
auto bind_this(R (U::*p)(Args...), U * pp)
-> decltype(bind_this_sub(p, pp, make_int_sequence< sizeof...(Args) >{}))
{
  return bind_this_sub(p, pp, make_int_sequence< sizeof...(Args) >{});
}

// utility
template<class R, class U, class... Args, int... Is>
auto bind_this_sub(R (U::*p)(Args...) const, U * pp, int_sequence<Is...>)
-> decltype(std::bind(p, pp, std::placeholder_template<Is>{}...))
{
  return std::bind(p, pp, std::placeholder_template<Is>{}...);
}

// binds a member function only for the this pointer using std::bind
template<class R, class U, class... Args>
auto bind_this(R (U::*p)(Args...) const, U * pp)
-> decltype(bind_this_sub(p, pp, make_int_sequence< sizeof...(Args) >{}))
{
  return bind_this_sub(p, pp, make_int_sequence< sizeof...(Args) >{});
}
#endif //RRTS_BIND_THIS_H
