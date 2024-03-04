use alloc::vec::Vec;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::Drawable;
use embedded_graphics::geometry::{Dimensions, Point, Size};
use embedded_graphics::image::{Image, ImageDrawable};
use embedded_graphics::mono_font::{MonoFont, MonoTextStyle};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::Primitive;
use embedded_graphics::primitives::{PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, RoundedRectangle};
use embedded_graphics::text::{Text, TextStyle};
use embedded_graphics::text::renderer::TextRenderer;
use heapless::String;
use profont::PROFONT_24_POINT;

pub struct GraphicUtils;

impl GraphicUtils {
    pub fn display_text<D, S>(display: &mut D, pos: Point, character_style: S,
                              text_style: TextStyle, text: &str) -> Result<Point, D::Error>
        where D: DrawTarget<Color=Rgb565>, S: TextRenderer<Color=Rgb565> {
        Text::with_text_style(
            text,
            pos,
            character_style,
            text_style,
        )
            .draw(display)
    }

    pub fn display_text_with_background<D, S>(display: &mut D, pos: Point, character_style: S,
                                              text_style: TextStyle, text: &str,
                                              background_style: PrimitiveStyle<Rgb565>,
                                              width: u32) -> Result<Point, D::Error>
        where D: DrawTarget<Color=Rgb565>, S: TextRenderer<Color=Rgb565> {
        let text = Text::with_text_style(
            text,
            pos,
            character_style,
            text_style,
        );
        Rectangle::new(pos, Size::new(width as u32, text.bounding_box().size.height))
            .into_styled(background_style)
            .draw(display)?;
        text.draw(display)
    }

    pub fn display_image_with_background<D, T>(display: &mut D, image: &Image<T>,
                                               background_style: PrimitiveStyle<Rgb565>) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565>, T: ImageDrawable<Color=Rgb565> {
        let bounding_box = image.bounding_box();
        Rectangle::new(bounding_box.top_left, bounding_box.size)
            .into_styled(background_style)
            .draw(display)?;

        image.draw(display)
    }

    pub fn get_button_size() -> Size {
        Size::new(90, 50)
    }
}

pub trait ListItem {
    fn get_text(&self) -> String<256>;
    fn get_height(&self) -> u16;
    fn get_font(&self) -> &MonoFont<'_>;
    fn get_text_style(&self) -> TextStyle;
}

pub struct List<T> {
    list_items: Vec<T>,
    pos: Point,
    size: Size,
    selected_index: usize,
    visible_lines: usize,
    window_start: usize,
    highlight_color: Rgb565,
    background_color: Rgb565,
    text_color: Rgb565
}

impl<T: ListItem + Clone> List<T> {
    pub fn new(items: &Vec<T>, pos: Point, size: Size, theme: &Theme) -> Self {
        List {
            list_items: items.clone(),
            pos,
            size,
            selected_index: 0,
            visible_lines: (size.height as u16 / items.first().unwrap().get_height()) as usize,
            window_start: 0,
            highlight_color: theme.highlight_color,
            background_color: theme.screen_background_color,
            text_color: theme.text_color_primary
        }
    }

    fn get_selected_style(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyleBuilder::new()
            .fill_color(self.highlight_color)
            .build()
    }

    fn get_background_style(&self) -> PrimitiveStyle<Rgb565> {
        PrimitiveStyleBuilder::new()
            .fill_color(self.background_color)
            .build()
    }

    fn get_character_style<'a>(&self, item: &'a T) -> MonoTextStyle<'a, Rgb565> {
        MonoTextStyle::new(
            item.get_font(),
            self.text_color)
    }

    pub fn draw<D>(&self, display: &mut D) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        for list_items_index in self.window_start..(self.window_start + self.visible_lines).min(self.list_items.len()) {
            let text = self.list_items[list_items_index].get_text();
            let item_height = self.list_items[list_items_index].get_height();
            let character_style = self.get_character_style(&self.list_items[list_items_index]);
            let text_style = self.list_items[list_items_index].get_text_style();
            let mut background_style = self.get_background_style();
            if self.selected_index == list_items_index {
                background_style = self.get_selected_style();
            }

            GraphicUtils::display_text_with_background(display, Point::new(self.pos.x, self.pos.y + ((list_items_index - self.window_start) * item_height as usize) as i32),
                                                       character_style, text_style, text.as_str(), background_style, self.size.width)?;
        }
        Ok(())
    }

    pub fn scroll_down<D>(&mut self, display: &mut D) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        if self.selected_index < self.list_items.len() {
            self.selected_index += 1
        };
        if self.selected_index > self.window_start + self.visible_lines - 1 {
            self.window_start += 1;
        }

        self.draw(display)
    }

    pub fn scroll_up<D>(&mut self, display: &mut D) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        if self.selected_index > 0 {
            self.selected_index -= 1
        };
        if self.selected_index < self.window_start {
            self.window_start -= 1;
        }

        self.draw(display)
    }

    pub fn select_at_pos<D>(&mut self, display: &mut D, pos: Point) -> Result<usize, D::Error>
        where D: DrawTarget<Color=Rgb565> {
        for list_items_index in self.window_start..(self.window_start + self.visible_lines).min(self.list_items.len()) {
            let item_height = self.list_items[list_items_index].get_height();
            let item_pos = Point::new(self.pos.x, self.pos.y + ((list_items_index - self.window_start) * item_height as usize) as i32);
            let bounding_box = Rectangle::new(item_pos, Size::new(self.size.width, item_height as u32));
            if (bounding_box.contains(pos)) {
                self.selected_index = list_items_index;
                break;
            }
        }
        self.draw(display)?;
        Ok(self.selected_index)
    }

    pub fn get_selected_index(&self) -> usize {
        self.selected_index
    }

    pub fn set_selected_index(&mut self, index: usize) {
        if index >= 0 && index < self.list_items.len() {
            self.selected_index = index;
        }
    }

    pub fn get_bounding_box(&self) -> Rectangle {
        Rectangle::new(self.pos, self.size)
    }
}

pub struct Button<'a, T> {
    image: &'a T,
    pos: Point,
    size: Size,
}

impl<'a, T: ImageDrawable<Color=Rgb565>> Button<'a, T> {
    pub fn new(image_drawable: &'a T, position: Point) -> Self {
        Button {
            image: image_drawable,
            pos: position,
            size: GraphicUtils::get_button_size(),
        }
    }

    pub fn draw<D>(&self, display: &mut D, background_style: PrimitiveStyle<Rgb565>) -> Result<(), D::Error>
        where D: DrawTarget<Color=Rgb565> {
        let visible_pos = Point::new(self.pos.x + 5, self.pos.y + 5);
        let visible_size = Size::new(self.size.width - 10, self.size.height - 10);
        RoundedRectangle::with_equal_corners(Rectangle::new(visible_pos, visible_size), Size::new(10, 10))
            .into_styled(background_style)
            .draw(display)?;

        let image_margin_x = (visible_size.width - self.image.size().width) / 2;
        let image_margin_y = (visible_size.height - self.image.size().height) / 2;

        let image = Image::new(self.image, Point::new(visible_pos.x + image_margin_x as i32, visible_pos.y + image_margin_y as i32));
        image.draw(display)
    }
}

pub struct Theme {
    pub button_background_color: Rgb565,
    pub button_foreground_color: Rgb565,
    pub screen_background_color: Rgb565,
    pub text_color_primary: Rgb565,
    pub highlight_color: Rgb565,
}
