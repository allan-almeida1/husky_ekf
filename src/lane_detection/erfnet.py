from keras.layers import Conv2D, Conv2DTranspose, BatchNormalization, MaxPool2D, Activation, Add, concatenate
from keras import layers, Model

class ERFNet:

    def __init__(self, input_shape):
        self.input_shape = input_shape


    def NonBottleNeck1D(self, input_tensor, filters, dilatation_rate):
        x = Conv2D(filters=filters, kernel_size=(3, 1), padding='same', activation='relu')(input_tensor)
        x = Conv2D(filters=filters, kernel_size=(1, 3), padding='same')(x)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)
        x = Conv2D(filters=filters, kernel_size=(3, 1), padding='same', dilation_rate=(dilatation_rate, 1))(x)
        x = Conv2D(filters=filters, kernel_size=(1, 3), padding='same', dilation_rate=(1, dilatation_rate))(x)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)
        x = Add()([x, input_tensor])
        x = Activation('relu')(x)
        return x

    def DownsamplerBlock(self, input_tensor, ch_out, ch_in):
        x1 = Conv2D(filters=ch_out - ch_in, kernel_size=(3, 3), strides=2, padding='same')(input_tensor)
        x2 = MaxPool2D(pool_size=(2, 2), strides=2)(input_tensor)
        x = concatenate([x1, x2], axis=-1)
        x = BatchNormalization()(x)
        return x

    def UpsamplerBlock(self, input_tensor, ch_out):
        x = Conv2DTranspose(filters=ch_out, kernel_size=(3, 3), strides=2, padding='same')(input_tensor)
        x = BatchNormalization()(x)
        x = Activation('relu')(x)
        return x
    
    def build(self):
        input = layers.Input(shape=self.input_shape)

        x = self.DownsamplerBlock(input, 16, 3)
        x = self.DownsamplerBlock(x, 64, 16)

        for i in range(5):
            x = self.NonBottleNeck1D(x, 64, 1)

        x = self.DownsamplerBlock(x, 128, 64)

        for i in range(2):
            x = self.NonBottleNeck1D(x, 128, 2)
            x = self.NonBottleNeck1D(x, 128, 4)
            x = self.NonBottleNeck1D(x, 128, 8)
            x = self.NonBottleNeck1D(x, 128, 16)

        x = self.UpsamplerBlock(x, 64)

        for i in range(2):
            x = self.NonBottleNeck1D(x, 64, 1)
            x = self.NonBottleNeck1D(x, 64, 1)

        x = self.UpsamplerBlock(x, 16)

        for i in range(2):
            x = self.NonBottleNeck1D(x, 16, 1)
            x = self.NonBottleNeck1D(x, 16, 1)

        x = Conv2DTranspose(filters=1, kernel_size=(3, 3), strides=2, padding='same')(x)
        x = Activation('sigmoid')(x)

        model = Model(input, x)
        return model