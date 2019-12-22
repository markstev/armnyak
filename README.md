# armnyak

Command to generate protos:
protoc -I=stepper_hat/proto -I=nanopb/generator/proto --python_out=protoc stepper_hat/proto/lcd.proto
