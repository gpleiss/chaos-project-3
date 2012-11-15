function res = function_reducer(f, Val, index)
    res = f(Val);
    res = res(index);
end