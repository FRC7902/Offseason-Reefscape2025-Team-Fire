package frc.robot.subsystems;

public class Main {
    // what is a class
    // classes are blueprints
    // classes are used to create objects

    //encapsulation
    public static void main(String[] args) {
        int a = 5;
        Dog Ryder = new Dog(true,true,720,"Ryder",1);
        if (a == 5){
            System.out.println("Ryder is a good boy");
        }
        else if (a == 6){
            System.out.println("FRC is very cool");
        }
        else {
            System.out.println("If not given enough attention, Ryder will be sad");
        }
        switch (a){
            case 5:
                System.out.println("Ryder is a good boy");
                break;
            case 6:
                System.out.println("Ryder ate lunch today");
                break;
            default:
                System.out.println("Programming is the best subdivision");
        }
        for (int i = 0; i < 9; i++){
            System.out.println("Programming is the best subdivision");
        }
        int[] arr = new int[5];
        for (int i : arr){
            System.out.println(i);
        }
       
    }
}
class Dog {
    // attributes
    // haveFood
    // haveWater
    // attention
    // name
    // age
    boolean haveFood;
    boolean haveWater;
    int attention;
    String name;
    int age;
    // in minutes
    public Dog(boolean haveFood, boolean haveWater, int attention, String name, int age) {
        this.haveFood = haveFood;
        this.haveWater = haveWater;
        this.attention = attention;
        this.name = name;
        this.age = age;
    }
    // behaviours
    // methods
    public void run() {
        System.out.println(haveFood);
        System.out.print("runnnnning");
        System.out.print("runnnnning");
        //Output: runnnningrunnning
        System.out.println("runnnnning");
        System.out.print("runnnnning");
        //runnnnnnning
        //runnnnnnning
    }
    // run
    // bite
    public String bite() {
        return "biteeeee";
    }
}
// eclipse
// source code -> byte code
// compiled
// byte code -> machine code
// interpreted
// VIM
// IntellJ