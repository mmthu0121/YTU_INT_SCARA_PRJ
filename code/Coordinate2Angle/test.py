import cod2ang

x = input("What is your x? ");
y = input("What is your y? ");       
l1 = input("What is your l1? ");       
l2 = input("What is your l2? ");       

[a1,a2] = cod2ang.cod2ang(x,y,l1,l2);

print("degree of 1st link is ", a1);
print("degree of 2nd link is ", a2);
